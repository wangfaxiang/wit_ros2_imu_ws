#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU 三维姿态可视化节点。

订阅 sensor_msgs/Imu（默认 /imu/data），把 IMU 的 orientation 实时映射到一个
三维模型上，并在 RViz2 中显示：

    * visualization_msgs/MarkerArray —— 3D 模型（默认为几何体拼装，也支持自定义 mesh）
    * TF  <fixed_frame> -> <model_frame> —— 便于同时在 RViz2 中使用 TF/Axes/RobotModel

用法示例：
    ros2 run wit_ros2_imu imu_visualizer
    ros2 run wit_ros2_imu imu_visualizer --ros-args -p imu_topic:=/imu/data
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# ==========================================================
# 四元数工具函数 (x, y, z, w)
# ==========================================================
def quaternionNormalize(q):
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def quaternionSlerp(q0, q1, t):
    """球面线性插值，t=0 返回 q0，t=1 返回 q1。"""
    dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]

    # 保证走最短路径
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot

    if dot > 0.9995:      # 足够接近，退化为线性插值
        return quaternionNormalize(tuple(a + t * (b - a) for a, b in zip(q0, q1)))

    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * t
    sinTheta = math.sin(theta)
    sinTheta0 = math.sin(theta0)

    s0 = math.cos(theta) - dot * sinTheta / sinTheta0
    s1 = sinTheta / sinTheta0

    return quaternionNormalize(
        tuple(s0 * a + s1 * b for a, b in zip(q0, q1))
    )


def yawQuaternion(yaw):
    """绕 Z 轴的旋转四元数。"""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def pitchQuaternion(pitch):
    """绕 Y 轴的旋转四元数。"""
    half = pitch * 0.5
    return (0.0, math.sin(half), 0.0, math.cos(half))


# Marker.ARROW 默认沿本地 +X 方向，这里给出三条互相正交的
# 单位轴 (X/Y/Z) 对应的姿态四元数。
AXIS_QUATERNIONS = {
    "x": (0.0, 0.0, 0.0, 1.0),                        # 绕 Z 轴   0°  -> +X
    "y": yawQuaternion(math.pi / 2.0),                # 绕 Z 轴 +90°  -> +Y
    "z": pitchQuaternion(-math.pi / 2.0),             # 绕 Y 轴 -90°  -> +Z
}


class ImuVisualizerNode(Node):
    """订阅 IMU 数据并在 RViz2 中显示三维模型实时位姿。"""

    def __init__(self):
        super().__init__("imu_visualizer_node")

        # ---------------- 参数 ----------------
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("marker_topic", "/imu_model/markers")
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("model_frame", "imu_model")
        self.declare_parameter("publish_tf", True)

        # 模型外观
        self.declare_parameter("mesh_resource", "")
        self.declare_parameter("mesh_scale", 1.0)
        self.declare_parameter("body_length", 0.20)
        self.declare_parameter("body_width", 0.12)
        self.declare_parameter("body_height", 0.04)

        # 坐标轴
        self.declare_parameter("show_axes", True)
        self.declare_parameter("axis_length", 0.25)
        self.declare_parameter("axis_shaft_diameter", 0.012)
        self.declare_parameter("axis_head_diameter", 0.030)

        # 其他
        self.declare_parameter("smoothing", 1.0)          # 1.0=不平滑，越小越平滑
        self.declare_parameter("marker_lifetime", 0.5)    # 秒，0 表示永久

        self.imuTopic = self.get_parameter("imu_topic").value
        self.markerTopic = self.get_parameter("marker_topic").value
        self.fixedFrame = self.get_parameter("fixed_frame").value
        self.modelFrame = self.get_parameter("model_frame").value
        self.publishTf = bool(self.get_parameter("publish_tf").value)

        self.meshResource = self.get_parameter("mesh_resource").value
        self.meshScale = float(self.get_parameter("mesh_scale").value)
        self.bodyLength = float(self.get_parameter("body_length").value)
        self.bodyWidth = float(self.get_parameter("body_width").value)
        self.bodyHeight = float(self.get_parameter("body_height").value)

        self.showAxes = bool(self.get_parameter("show_axes").value)
        self.axisLength = float(self.get_parameter("axis_length").value)
        self.axisShaftDiameter = float(self.get_parameter("axis_shaft_diameter").value)
        self.axisHeadDiameter = float(self.get_parameter("axis_head_diameter").value)

        self.smoothing = max(0.01, min(1.0, float(self.get_parameter("smoothing").value)))
        self.markerLifetime = float(self.get_parameter("marker_lifetime").value)

        # ---------------- 状态 ----------------
        self.lastQuaternion = None

        # ---------------- 接口 ----------------
        self.imuSub = self.create_subscription(Imu, self.imuTopic, self.imuCallback, 10)
        self.markerPub = self.create_publisher(MarkerArray, self.markerTopic, 10)
        self.tfBroadcaster = TransformBroadcaster(self) if self.publishTf else None

        self.get_logger().info(
            f"IMU 可视化节点已启动: 订阅 {self.imuTopic} -> 发布 {self.markerTopic} "
            f"(frame: {self.fixedFrame} -> {self.modelFrame})"
        )

    # ==========================================================
    # IMU 回调
    # ==========================================================
    def imuCallback(self, msg: Imu):
        # orientation_covariance[0] < 0 表示该消息不包含有效姿态
        if msg.orientation_covariance[0] < 0.0:
            self.get_logger().warn(
                "IMU 消息中没有有效姿态 (orientation_covariance[0] < 0)，已忽略",
                throttle_duration_sec=5.0,
            )
            return

        quat = quaternionNormalize((
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ))

        # 低通平滑，避免抖动
        if self.lastQuaternion is not None and self.smoothing < 1.0:
            quat = quaternionSlerp(self.lastQuaternion, quat, self.smoothing)
        self.lastQuaternion = quat

        # 时间戳：消息为空时退化为当前时间
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        else:
            stamp = msg.header.stamp

        if self.publishTf:
            self.publishTransform(quat, stamp)

        self.publishMarkers(quat, stamp)

    # ==========================================================
    # TF: fixed_frame -> model_frame（只有姿态，无平移）
    # ==========================================================
    def publishTransform(self, quat, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.fixedFrame
        tf.child_frame_id = self.modelFrame

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]

        self.tfBroadcaster.sendTransform(tf)

    # ==========================================================
    # MarkerArray
    # ==========================================================
    def publishMarkers(self, quat, stamp):
        markers = MarkerArray()

        # ---- 主体（mesh 或几何体）----
        markers.markers.append(self.buildBodyMarker(stamp))

        # ---- 三条坐标轴箭头（RGB = XYZ，互相正交）----
        if self.showAxes:
            markers.markers.append(
                self.buildAxisMarker(1, stamp, (1.0, 0.2, 0.2), AXIS_QUATERNIONS["x"])
            )
            markers.markers.append(
                self.buildAxisMarker(2, stamp, (0.2, 0.9, 0.2), AXIS_QUATERNIONS["y"])
            )
            markers.markers.append(
                self.buildAxisMarker(3, stamp, (0.3, 0.5, 1.0), AXIS_QUATERNIONS["z"])
            )

        self.markerPub.publish(markers)

    def buildBodyMarker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.modelFrame
        marker.header.stamp = stamp
        marker.ns = "imu_model"
        marker.id = 0
        marker.action = Marker.ADD

        marker.pose.position = Point(x=0.0, y=0.0, z=0.0)
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        if self.meshResource:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = self.meshResource
            marker.mesh_use_embedded_materials = True
            marker.scale = Vector3(
                x=self.meshScale, y=self.meshScale, z=self.meshScale
            )
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        else:
            marker.type = Marker.CUBE
            marker.scale = Vector3(
                x=self.bodyLength, y=self.bodyWidth, z=self.bodyHeight
            )
            marker.color = ColorRGBA(r=0.25, g=0.55, b=0.90, a=0.95)

        self.applyLifetime(marker)
        return marker

    def buildAxisMarker(self, markerId, stamp, rgb, orientation) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.modelFrame
        marker.header.stamp = stamp
        marker.ns = "imu_axes"
        marker.id = markerId
        marker.action = Marker.ADD
        marker.type = Marker.ARROW

        marker.pose.position = Point(x=0.0, y=0.0, z=0.0)
        marker.pose.orientation = Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
        )

        marker.scale = Vector3(
            x=self.axisLength,
            y=self.axisShaftDiameter,
            z=self.axisHeadDiameter,
        )
        marker.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)

        self.applyLifetime(marker)
        return marker

    def applyLifetime(self, marker: Marker):
        if self.markerLifetime > 0.0:
            sec = int(self.markerLifetime)
            nsec = int((self.markerLifetime - sec) * 1e9)
            marker.lifetime.sec = sec
            marker.lifetime.nanosec = nsec


def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
