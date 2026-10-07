#!/usr/bin/env python3
"""goto_node：根据地点名发送 Nav2 导航目标（支持多地点连续导航 + 卡住检测）。

用法：
  1. 先启动导航（navigation2_rviz.launch.py 或 slam_nav2.launch.py）
  2. 运行本节点
  3. 发布地点名到 /goto_place（单个或逗号分隔的多个）：
       ros2 topic pub --once /goto_place std_msgs/msg/String "data: '客厅'"
       ros2 topic pub --once /goto_place std_msgs/msg/String "data: '厨房,客厅'"

连续导航：按顺序依次导航，到达一个自动去下一个。
卡住检测：如果 10 秒内小车位置没变化（原地不动），跳过当前地点去下一个。

地点坐标在 config/places.yaml 里手动填。
"""
import math
import os

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class GotoNode(Node):
    def __init__(self):
        super().__init__('goto_node')

        # 地点坐标表：默认读源码目录下的 places.yaml，改完直接生效（不用重编译）
        default_file = os.path.join(
            os.path.expanduser('~'),
            '-ros2--main/软件/1.源码/xuegeros_ws/src/xuegecar_voice_nav/config/places.yaml')
        self.declare_parameter('places_file', default_file)
        self._places_file = self.get_parameter('places_file').value
        # 原地不动多少秒算卡住（默认 10 秒）
        self.declare_parameter('stuck_timeout', 10.0)
        self._stuck_timeout = self.get_parameter('stuck_timeout').value

        # 导航 action 客户端
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 订阅地点指令
        self.create_subscription(String, 'goto_place', self._on_command, 10)
        # 到达通知：到达某地点后发布地点名到 /arrived
        self._arrived_pub = self.create_publisher(String, 'arrived', 10)
        # 卡住检测：订阅 /odom 跟踪位置 + 定时器检查
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(1.0, self._check_stuck)

        # 连续导航状态
        self._goal_queue = []       # 待导航的 (name, x, y, yaw)
        self._current_place = None  # 当前导航的地点名（用于到达播报）
        self._navigating = False
        self._goal_handle = None    # 当前目标 handle（用于取消）
        self._last_move_time = None
        self._last_pos = None

        self.get_logger().info(f'地点文件：{self._places_file}')
        self.get_logger().info(
            f'当前地点：{list(self._load_places().keys())}；'
            f'发布地点名到 /goto_place 即可导航（支持逗号分隔多个）')

    def _load_places(self):
        with open(self._places_file, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}
        places = {}
        for name, v in raw.items():
            if not isinstance(v, dict):
                self.get_logger().warn(f'地点「{name}」格式不对，已跳过')
                continue
            places[name] = (
                float(v.get('x', 0.0)),
                float(v.get('y', 0.0)),
                float(v.get('yaw', 0.0)),
            )
        return places

    def _parse_place(self, text):
        """把 '去客厅' / '去客厅。' / '客厅' 统一成 '客厅'。"""
        text = text.strip()
        for ch in '。，、！？；：,.!?;:':
            text = text.replace(ch, '')
        text = text.strip()
        for prefix in ('前往', '开到', '去', '到'):
            if text.startswith(prefix):
                text = text[len(prefix):]
                break
        return text.strip()

    def _on_command(self, msg):
        # 每次收到指令都重新读文件，改了 places.yaml 不用重启节点
        places = self._load_places()
        # 支持逗号分隔的多个地点："厨房,客厅"
        names = [self._parse_place(p) for p in msg.data.split(',')]
        names = [p for p in names if p]
        if not names:
            return

        self._goal_queue = []
        for name in names:
            if name not in places:
                self.get_logger().error(
                    f'未找到地点「{name}」，已知地点：{list(places.keys())}')
                return
            x, y, yaw = places[name]
            self._goal_queue.append((name, x, y, yaw))

        self.get_logger().info(f'收到指令：{names}，开始连续导航')
        self._send_next_goal()

    def _send_next_goal(self):
        if not self._goal_queue:
            self.get_logger().info('✅ 所有地点导航完成')
            return
        name, x, y, yaw = self._goal_queue.pop(0)
        self._current_place = name
        self.get_logger().info(f'前往「{name}」')
        self._send_goal(x, y, yaw)

    def _send_goal(self, x, y, yaw):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('导航服务 (navigate_to_pose) 未就绪，请先启动导航')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)
        # 重置卡住检测的基准
        self._navigating = True
        self._last_move_time = self.get_clock().now()
        self._last_pos = None
        self.get_logger().info(f'已发送导航目标 ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝，跳过')
            self._navigating = False
            self._send_next_goal()
            return
        self._goal_handle = goal_handle
        self.get_logger().info('目标已接受，开始导航…')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 到达目标')
            if self._current_place:
                msg = String()
                msg.data = self._current_place
                self._arrived_pub.publish(msg)
                self.get_logger().info(f'已发布到达通知：{self._current_place}')
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('目标被取消（卡住跳过）')
        else:
            self.get_logger().warn(f'导航结束，状态码 {result.status}')
        # 当前目标结束，发下一个
        self._navigating = False
        self._goal_handle = None
        self._send_next_goal()

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        if self._last_pos is None:
            self._last_pos = (pos.x, pos.y)
            self._last_move_time = self.get_clock().now()
            return
        # 位置变化超过 5cm 就算「动了」
        if abs(pos.x - self._last_pos[0]) + abs(pos.y - self._last_pos[1]) > 0.05:
            self._last_pos = (pos.x, pos.y)
            self._last_move_time = self.get_clock().now()

    def _check_stuck(self):
        if not self._navigating or self._last_move_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_move_time).nanoseconds / 1e9
        if elapsed > self._stuck_timeout:
            self.get_logger().warn(
                f'检测到 {self._stuck_timeout:.0f} 秒未移动，跳过当前地点')
            self._cancel_current()

    def _cancel_current(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _feedback_cb(self, msg):
        d = msg.feedback.distance_remaining
        self.get_logger().info(
            f'剩余距离 {d:.2f} m', throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = GotoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
