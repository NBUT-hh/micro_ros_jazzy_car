#!/usr/bin/env python3
"""skills.py：各种「能力」函数，供节点复用。

包含：ASR 语音识别、LLM 语义理解、TTS 语音合成、视觉描述、JSON 提取。
这些都是「纯能力」——输入配置和参数，返回结果，不依赖 ROS2 节点。

用到的 DashScope 全局配置（api_key / base_http_api_url）由节点在启动时设好，
这里直接用 dashscope 的全局默认值。
"""
import io
import json
import threading

import sounddevice as sd
import soundfile as sf

# TTS 播放锁：避免多个线程（主循环 + 到达播报）同时出声
_speak_lock = threading.Lock()


def recognize_speech(cfg, wav_file):
    """语音识别（DashScope Paraformer 一句话识别），返回识别文本。"""
    from dashscope.audio.asr import Recognition, RecognitionCallback

    recognition = Recognition(
        model=cfg.get('model', 'paraformer-realtime-v2'),
        callback=RecognitionCallback(),
        format='wav',
        sample_rate=16000,
    )
    result = recognition.call(wav_file)
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


def llm_understand(cfg, place_names, text):
    """LLM 语义理解（Qwen），返回 (地点列表, 是否视觉, 回复)。"""
    system = (
        f'你是一个机器人语音助手。已知地点：{place_names}。\n'
        '用户的话是语音识别的结果，可能带有口语或错别字。\n'
        '请判断用户意图：\n'
        '1. 想去某地（导航）：按顺序提取所有目标地点，visual 为 false。\n'
        '2. 想看/描述周围环境（如"看看前面""描述一下""前面有什么"）：'
        'places 为空数组，visual 为 true。\n'
        '生成一句简短友好的回复（不超过15字）。\n'
        '只输出 JSON，格式：{"places": ["地点1"], "visual": false, "reply": "回复内容"}\n'
        '若没识别出地点或视觉意图，places 为空数组、visual 为 false，'
        'reply 提示用户重新说。'
    )
    from dashscope import Generation

    resp = Generation.call(
        model=cfg.get('llm_model', 'qwen-plus'),
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
    data = extract_json(content)
    places = data.get('places', [])
    if isinstance(places, str):
        places = [places]
    visual = bool(data.get('visual', False))
    return [p.strip() for p in places if p], visual, data.get('reply', '').strip()


def speak(cfg, text):
    """语音合成（TTS）并播放。"""
    with _speak_lock:
        from dashscope.audio.tts_v2 import AudioFormat, SpeechSynthesizer

        synthesizer = SpeechSynthesizer(
            model=cfg.get('tts_model', 'cosyvoice-v1'),
            voice=cfg.get('tts_voice', 'longxiaochun'),
            format=AudioFormat.WAV_16000HZ_MONO_16BIT,
        )
        audio = synthesizer.call(text)  # 非流式，直接返回 WAV 字节
        if not audio:
            return
        data, sr = sf.read(io.BytesIO(audio))
        sd.play(data, sr)
        sd.wait()


def describe_scene(cfg):
    """抓一帧相机画面，用视觉大模型（Qwen-VL）描述，返回描述文本。"""
    import cv2
    from dashscope import MultiModalConversation

    url = cfg.get('camera_url', 'http://10.253.14.141:81/')
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        cap.release()
        return '相机打不开，请检查相机是否开机'
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return '没有取到相机画面'

    img_path = '/tmp/camera_frame.jpg'
    cv2.imwrite(img_path, frame)

    messages = [{
        'role': 'user',
        'content': [
            {'image': f'file://{img_path}'},
            {'text': '用简短的中文描述这张图片里有什么。'},
        ],
    }]
    resp = MultiModalConversation.call(
        model=cfg.get('vision_model', 'qwen-vl-plus'),
        messages=messages,
    )
    if resp.status_code != 200:
        return f'视觉识别失败：{resp.code} {resp.message}'

    content = resp.output.choices[0].message.content
    # content 是列表，形如 [{'text': '...'}]
    if isinstance(content, list) and content:
        return content[0].get('text', '').strip()
    return '没识别出内容'


def extract_json(content):
    """从 LLM 返回内容里提取 JSON（容忍 markdown 代码块包裹）。"""
    content = (content or '').strip()
    if content.startswith('```'):
        content = content.strip('`')
        if content.startswith('json'):
            content = content[4:]
    start, end = content.find('{'), content.rfind('}')
    if start != -1 and end != -1:
        content = content[start:end + 1]
    return json.loads(content)
