#!/usr/bin/env python3
"""vision_node：YOLO 实时检测节点。

订阅 xuegecar_camera 发布的 /camera/image_raw/compressed -> YOLO 检测
-> 弹窗显示检测框 + 日志打印检测结果。

这样相机只有 xuegecar_camera 一个客户端，不会再因为多客户端崩溃。

用法：
  1. 先启动 xuegecar_camera（唯一相机客户端）
     ros2 launch xuegecar_camera http_video_publisher.launch.py
  2. 再启动本节点
     ros2 run xuegecar_voice_nav vision_node

按 q 退出显示窗口。
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # YOLO 模型路径（默认用你本地已下载的模型）
        self.declare_parameter('model', '/home/huanghao/yolov8/yolo11n.pt')
        model_path = self.get_parameter('model').value

        # 订阅 xuegecar_camera 发布的压缩图像话题
        # 相机话题是 BEST_EFFORT，订阅必须也用 BEST_EFFORT，否则收不到
        video_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self._on_image, video_qos)
        self._latest = None

        self.get_logger().info(f'加载 YOLO 模型：{model_path}')
        self._model = YOLO(model_path)
        self.get_logger().info('模型加载完成，等待相机画面…')

        # 定时处理（10Hz）：解码 -> YOLO -> 显示
        self.create_timer(0.1, self._process)

    def _on_image(self, msg):
        self._latest = msg.data

    def _process(self):
        if self._latest is None:
            return
        # 解压 JPEG -> BGR 图
        arr = np.frombuffer(self._latest, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # YOLO 推理
        results = self._model(frame, verbose=False)
        annotated = results[0].plot()  # 带框+标签的图

        # 打印检测到的物体（置信度>=0.5，节流避免刷屏）
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if conf >= 0.5:
                    self.get_logger().info(
                        f'检测到 {self._model.names[cls]}（置信度 {conf:.2f}）',
                        throttle_duration_sec=2.0)

        cv2.imshow('YOLO 实时检测', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
