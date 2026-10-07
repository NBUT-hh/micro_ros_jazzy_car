#!/usr/bin/env python3
"""voice_word_node：唤醒词语音控制节点（编排层）。

流程：
  持续监听 -> sherpa-onnx KWS 检测唤醒词「小白」-> 自动录音 5 秒
  -> ASR 识别 -> LLM 理解 -> TTS 播报 / 视觉描述 -> 发布地点到 /goto_place

各能力（ASR / LLM / TTS / 视觉）在 skills.py 里，本节点只负责：
唤醒词检测、录音、编排调度。

用法：
  ros2 run xuegecar_voice_nav voice_word_node

依赖：
  sudo apt install libportaudio2
  pip install --user --break-system-packages dashscope sounddevice soundfile pyyaml numpy sherpa-onnx

配置在 config/asr.yaml 里（api_key / base_url / model / llm_model / tts_model / tts_voice / vision_model / camera_url / wake_word / wake_word_tokens / kws_dir）。
"""
import glob
import os
import queue
import threading

import numpy as np
import rclpy
import sounddevice as sd
import soundfile as sf
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from xuegecar_voice_nav import skills


class VoiceWordNode(Node):
    def __init__(self):
        super().__init__('voice_word_node')
        self.pub = self.create_publisher(String, 'goto_place', 10)
        # 订阅到达通知，播报「XX已到达」
        self.create_subscription(String, 'arrived', self._on_arrived, 10)

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
        self.place_names = '、'.join(self.places.keys())

        # 配置 DashScope（全局 key + base_url，skills 里直接使用）
        import dashscope
        dashscope.api_key = self.cfg['api_key']
        dashscope.base_http_api_url = self.cfg['base_url'].rstrip('/')
        self.get_logger().info(
            f'ASR 配置已加载（base_url: {self.cfg["base_url"]}）')

        # 唤醒词 + KWS 模型
        self.wake_word = self.cfg.get('wake_word', '小白')
        self.kws_dir = self.cfg.get('kws_dir', '')
        self._init_kws()

    # ---------- 唤醒词检测（sherpa-onnx KWS）----------
    def _init_kws(self):
        import sherpa_onnx

        if not self.kws_dir or not os.path.isdir(self.kws_dir):
            raise RuntimeError(
                f'KWS 模型目录不存在：{self.kws_dir}，请在 asr.yaml 里配置 kws_dir')
        encoder = glob.glob(os.path.join(self.kws_dir, 'encoder-*.onnx'))[0]
        decoder = glob.glob(os.path.join(self.kws_dir, 'decoder-*.onnx'))[0]
        joiner = glob.glob(os.path.join(self.kws_dir, 'joiner-*.onnx'))[0]
        tokens = os.path.join(self.kws_dir, 'tokens.txt')

        # sherpa-onnx 用 keywords_file，格式为「拼音token @汉字」每行一个。
        wtokens = self.cfg.get('wake_word_tokens', '')
        if not wtokens:
            raise RuntimeError('请在 asr.yaml 配置 wake_word_tokens（唤醒词的拼音 token）')
        keywords_file = '/tmp/kws_keywords.txt'
        with open(keywords_file, 'w', encoding='utf-8') as f:
            f.write(f'{wtokens} @{self.wake_word}\n')

        self.kws = sherpa_onnx.KeywordSpotter(
            tokens=tokens, encoder=encoder, decoder=decoder, joiner=joiner,
            keywords_file=keywords_file,
            num_threads=2, max_active_paths=4, provider='cpu')
        self.get_logger().info(f'唤醒词检测已就绪：{self.wake_word}')

    # ---------- 阶段1：持续监听检测唤醒词 ----------
    def _wait_wake_word(self, samplerate=16000):
        block = 1600  # 0.1 秒
        q = queue.Queue()

        def cb(indata, frames, time_info, status):
            q.put(indata.copy())

        with sd.InputStream(samplerate=samplerate, channels=1,
                            dtype='int16', blocksize=block, callback=cb):
            self.get_logger().info(f'等待唤醒词「{self.wake_word}」…')
            stream = self.kws.create_stream()
            while rclpy.ok():
                chunk = q.get()
                samples = chunk[:, 0].astype(np.float32) / 32768.0
                stream.accept_waveform(samplerate, samples)
                while self.kws.is_ready(stream):
                    self.kws.decode_stream(stream)
                    if self.kws.get_result(stream):
                        return

    # ---------- 阶段2：录固定时长指令 ----------
    def _record(self, filename='/tmp/voice_word_cmd.wav',
                samplerate=16000, duration=5.0):
        block = 1600  # 0.1 秒
        record_blocks = int(samplerate * duration / block)
        q = queue.Queue()

        def cb(indata, frames, time_info, status):
            q.put(indata.copy())

        with sd.InputStream(samplerate=samplerate, channels=1,
                            dtype='int16', blocksize=block, callback=cb):
            self.get_logger().info(f'🎤 录音 {duration:.0f} 秒…')
            rec = []
            for _ in range(record_blocks):
                rec.append(q.get())
        audio = np.concatenate(rec)[:, 0]
        sf.write(filename, audio, samplerate)
        return filename

    # ---------- 判断识别文本是否有实际内容 ----------
    @staticmethod
    def _has_content(text):
        for ch in '。，、！？；：,.!?;: ':
            text = text.replace(ch, '')
        return len(text.strip()) > 0

    # ---------- 到达播报 ----------
    def _on_arrived(self, msg):
        place = msg.data.strip()
        if place:
            self.get_logger().info(f'收到到达通知：{place}')
            skills.speak(self.cfg, f'{place}已到达')

    # ---------- 主循环 ----------
    def run(self):
        wake_prompt = self.cfg.get('wake_prompt', '我在，请说')
        bye_prompt = self.cfg.get('bye_prompt', '有需要再叫我')

        while rclpy.ok():
            try:
                # 阶段1：等唤醒词
                self._wait_wake_word()
                self.get_logger().info('✅ 已唤醒')
                skills.speak(self.cfg, wake_prompt)

                # 阶段2：连续对话，录音没内容就退回等唤醒词
                while rclpy.ok():
                    wav = self._record()
                    text = skills.recognize_speech(self.cfg, wav)
                    self.get_logger().info(f'识别结果：「{text}」')

                    if not self._has_content(text):
                        self.get_logger().info('录音无内容，退回等待唤醒词')
                        skills.speak(self.cfg, bye_prompt)
                        break

                    places, visual, reply = skills.llm_understand(
                        self.cfg, self.place_names, text)
                    self.get_logger().info(
                        f'理解结果：地点={places or "(无)"} 视觉={visual} 回复={reply}')

                    if reply:
                        skills.speak(self.cfg, reply)

                    if visual:
                        desc = skills.describe_scene(self.cfg)
                        self.get_logger().info(f'视觉描述：{desc}')
                        skills.speak(self.cfg, desc)
                    elif places:
                        msg = String()
                        msg.data = ','.join(places)   # 逗号分隔的地点列表
                        self.pub.publish(msg)
                        self.get_logger().info(f'已发布地点到 /goto_place：{msg.data}')
                    # 循环继续：继续录音，不用再喊唤醒词
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'处理失败：{e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceWordNode()
    # 后台线程跑 rclpy.spin，处理 /arrived 等订阅回调
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
