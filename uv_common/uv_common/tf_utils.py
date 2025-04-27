import numpy as np
import math
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException
from geometry_msgs.msg import TransformStamped
from typing import Optional

def lookup_tf(tf_buffer: Buffer, source_frame: str, target_frame: str, stamp: Time) -> Optional[TransformStamped]:
    timeout_duration = Duration(seconds=0.1)
    if not tf_buffer.can_transform(target_frame, source_frame, stamp, timeout=timeout_duration):
        return None
    try:
        return tf_buffer.lookup_transform(target_frame, source_frame, stamp, timeout=timeout_duration)
    except TransformException:
        return None


def apply_tf_2d(points2d: np.ndarray, tf: TransformStamped) -> np.ndarray:
    q = tf.transform.rotation
    t = tf.transform.translation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    T = np.array([t.x, t.y])
    return points2d @ R.T + T