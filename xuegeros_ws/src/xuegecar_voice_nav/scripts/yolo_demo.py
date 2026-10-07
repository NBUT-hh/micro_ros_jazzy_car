#!/usr/bin/env python3
"""YOLO 实时检测演示（在 yolov8 conda 环境里运行，不依赖 ROS2）。

读取 ESP32-cam 图传流 -> YOLOv8 实时检测 -> 弹出窗口显示检测框。

用法：
  conda activate yolov8
  python3 /home/huanghao/-ros2--main/软件/1.源码/xuegeros_ws/src/xuegecar_voice_nav/scripts/yolo_demo.py

按 q 退出。
"""
import time

import cv2
from ultralytics import YOLO

# 相机图传地址（ESP32-cam 的 MJPEG 流）
CAMERA_URL = 'http://10.253.14.141:81/'
# 模型：yolov8n 最小最快；可换 yolov8s/m/l 更准但更慢
MODEL = '/home/huanghao/yolov8/yolo11n.pt'


def main():
    # 加载模型（首次运行会自动下载 yolov8n.pt，约 6MB）
    model = YOLO(MODEL)

    # 用 FFMPEG 后端读 MJPEG 流（默认后端解析不好会读成空帧）
    cap = cv2.VideoCapture(CAMERA_URL, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减小缓冲，降低延迟
    if not cap.isOpened():
        print(f'❌ 相机打不开，请检查相机是否开机、URL 是否正确：{CAMERA_URL}')
        return
    print('相机已连接，开始检测（按 q 退出）...')

    last = time.time()
    while True:
        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            print('读不到画面，可能相机断流')
            break
        if frame.shape[0] < 100 or frame.shape[1] < 100:
            print(f'画面异常（分辨率 {frame.shape[1]}x{frame.shape[0]}），跳过')
            continue

        # YOLO 推理（verbose=False 关掉控制台刷屏）
        results = model(frame, verbose=False)
        annotated = results[0].plot()  # 带框+标签的图

        # 实时 FPS
        now = time.time()
        fps = 1.0 / max(now - last, 1e-6)
        last = now
        cv2.putText(annotated, f'FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('YOLO 实时检测', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
