#!/usr/bin/env python3
"""voice_node：语音控制节点（阿里云百炼 DashScope 版，含 LLM 理解 + TTS 播报）。

流程：
  麦克风录音 -> DashScope ASR 识别 -> Qwen LLM 理解（提取地点 + 生成回复）
  -> TTS 语音播报回复 -> 发布地点到 /goto_place ->（goto_node 负责导航）

用法：
  ros2 run xuegecar_voice_nav voice_node
  运行后按回车开始录音，说"去客厅"，再按回车结束。

依赖：
  sudo apt install libportaudio2
  pip install --user --break-system-packages dashscope requests sounddevice soundfile pyyaml numpy

配置在 config/asr.yaml 里（api_key / base_url / model / llm_model / tts_model / tts_voice）。
"""
import io
import json
import os
import queue

import numpy as np
import rclpy
import sounddevice as sd
import soundfile as sf
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.pub = self.create_publisher(String, 'goto_place', 10)

        # 读 config/asr.yaml
        default_cfg = os.path.join(
            os.path.expanduser('~'),
            '-ros2--main/软件/1.源码/xuegeros_ws/src/xuegecar_voice_nav/config/asr.yaml')
        self.declare_parameter('asr_config', default_cfg)
        with open(self.get_parameter('asr_config').value, 'r',
                  encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)

        # 读 config/places.yaml（只取地点名，给 LLM 做提示词）
        default_places = os.path.join(
            os.path.expanduser('~'),
            '-ros2--main/软件/1.源码/xuegeros_ws/src/xuegecar_voice_nav/config/places.yaml')
        self.declare_parameter('places_file', default_places)
        with open(self.get_parameter('places_file').value, 'r',
                  encoding='utf-8') as f:
            self.places = {k: v for k, v in (yaml.safe_load(f) or {}).items()
                           if isinstance(v, dict)}

        # 配置 DashScope
        import dashscope
        dashscope.api_key = self.cfg['api_key']
        dashscope.base_http_api_url = self.cfg['base_url'].rstrip('/')
        self.get_logger().info(
            f'ASR 配置已加载（base_url: {self.cfg["base_url"]}）')

    # ---------- 录音 ----------
    def _record(self, filename='/tmp/voice_cmd.wav', samplerate=16000):
        input('>>> 按回车开始录音，说出指令后，再按回车结束…')
        self.get_logger().info('🎤 录音中…（再按回车结束）')
        q = queue.Queue()

        def cb(indata, frames, time_info, status):
            q.put(indata.copy())

        with sd.InputStream(samplerate=samplerate, channels=1,
                            dtype='int16', callback=cb):
            input()

        chunks = []
        while not q.empty():
            chunks.append(q.get())
        audio = np.concatenate(chunks) if chunks else np.zeros(0, dtype='int16')
        sf.write(filename, audio, samplerate)
        return filename

    # ---------- ASR 语音识别 ----------
    def recognize(self, filename):
        from dashscope.audio.asr import Recognition, RecognitionCallback

        recognition = Recognition(
            model=self.cfg.get('model', 'paraformer-realtime-v2'),
            callback=RecognitionCallback(),
            format='wav',
            sample_rate=16000,
        )
        result = recognition.call(filename)
        if getattr(result, 'status_code', None) != 200:
            raise RuntimeError(
                f'ASR 失败：code={getattr(result, "code", "")} '
                f'msg={getattr(result, "message", "")}')

        text = ''
        sentence = None
        if callable(getattr(result, 'get_sentence', None)):
            sentence = result.get_sentence()
        if isinstance(sentence, list):
            text = ''.join(
                s.get('text', '') for s in sentence if isinstance(s, dict))
        elif isinstance(sentence, dict):
            text = sentence.get('text', '')
        elif isinstance(sentence, str):
            text = sentence
        if not text and isinstance(getattr(result, 'output', None), dict):
            out = result.output
            sent = out.get('sentence')
            if isinstance(sent, dict):
                text = sent.get('text', '')
            elif isinstance(sent, str):
                text = sent
            elif isinstance(out.get('text'), str):
                text = out['text']
        return text.strip()

    # ---------- LLM 语义理解 ----------
    def _llm_parse(self, text):
        """用 Qwen 理解用户话，返回 (地点名, 回复)。"""
        place_names = '、'.join(self.places.keys())
        system = (
            f'你是一个机器人语音助手。已知地点：{place_names}。\n'
            '用户的话是语音识别的结果，可能带有口语或错别字。\n'
            '请从用户的话里提取目标地点（必须是已知地点之一），'
            '并生成一句简短友好的回复（不超过15字）。\n'
            '只输出 JSON，格式：{"place": "地点名", "reply": "回复内容"}\n'
            '若用户没指定地点、或地点不在已知列表里，place 设为空字符串，'
            'reply 提示用户重新说。'
        )
        from dashscope import Generation

        resp = Generation.call(
            model=self.cfg.get('llm_model', 'qwen-plus'),
            messages=[
                {'role': 'system', 'content': system},
                {'role': 'user', 'content': text},
            ],
            result_format='message',
            temperature=0.1,
        )
        if resp.status_code != 200:
            raise RuntimeError(
                f'LLM 失败：code={resp.code} msg={resp.message}')
        content = resp.output.choices[0].message.content
        data = self._extract_json(content)
        return data.get('place', '').strip(), data.get('reply', '').strip()

    @staticmethod
    def _extract_json(content):
        content = (content or '').strip()
        if content.startswith('```'):
            content = content.strip('`')
            if content.startswith('json'):
                content = content[4:]
        start, end = content.find('{'), content.rfind('}')
        if start != -1 and end != -1:
            content = content[start:end + 1]
        return json.loads(content)

    # ---------- TTS 语音播报 ----------
    def _speak(self, text):
        from dashscope.audio.tts_v2 import AudioFormat, SpeechSynthesizer

        synthesizer = SpeechSynthesizer(
            model=self.cfg.get('tts_model', 'cosyvoice-v1'),
            voice=self.cfg.get('tts_voice', 'longxiaochun'),
            format=AudioFormat.WAV_16000HZ_MONO_16BIT,
        )
        audio = synthesizer.call(text)  # 非流式，直接返回 WAV 字节
        if not audio:
            self.get_logger().warn('TTS 无音频返回')
            return
        data, sr = sf.read(io.BytesIO(audio))
        sd.play(data, sr)
        sd.wait()

    # ---------- 主循环 ----------
    def run(self):
        while rclpy.ok():
            try:
                wav = self._record()
                text = self.recognize(wav)
                self.get_logger().info(f'识别结果：「{text}」')
                if not text:
                    continue

                place, reply = self._llm_parse(text)
                self.get_logger().info(f'理解结果：地点={place or "(无)"} 回复={reply}')

                if reply:
                    self._speak(reply)

                if place:
                    msg = String()
                    msg.data = place
                    self.pub.publish(msg)
                    self.get_logger().info(f'已发布地点到 /goto_place：{place}')
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'处理失败：{e}')
                try:
                    self._speak('出错了，请再说一遍')
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
