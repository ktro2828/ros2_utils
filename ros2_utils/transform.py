import argparse
from typing import Any, List, Optional, Tuple

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from ros2_utils.rosbag import BagFileParser


class TransformInterface(Node):
    """Parse bag file and transform recognized objects from base_link to map coordinate system."""

    def __init__(self, bag_file: str) -> None:
        """
        Args:
            bag_file (str): Bag file path (.db3).
        """
        super().__init__("transform_interface")
        self.parser: BagFileParser = BagFileParser(bag_file)

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.set_transform()

    def set_transform(self):
        tf_msgs: List[Tuple[int, TFMessage]] = self.parser.get_messages("/tf")
        for _, msg in tf_msgs:
            for trans in msg.transforms:
                self.tf_buffer.set_transform(trans, "default_authority")

    def get_transform(self, topic_name: str, target_frame: str) -> List[Tuple[int, TransformStamped]]:
        """
        Args:
            topic_name (str): Name of topic.
            target_frame (str): Name of target frame.
        Returns:
            ret (List[Tuple[Optional[TransformStamped], Any]]): List of transforms and messages.
                If lookup_transform() is failed, (None, message) will be set.
        """
        target_msgs: List[Tuple[int, Any]] = self.parser.get_messages(topic_name)
        ret: List[Tuple[Optional[TransformStamped], Any]] = []
        for _, msg in target_msgs:
            try:
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                )
                ret.append((trans, msg))
            except Exception as e:
                print(e)
                # ret.append((None, msg))
                pass

        return ret


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-f", "--filepath", type=str, help="Bag file path(.db3)")
    args = arg_parser.parse_args()

    rclpy.init()
    transformer = TransformInterface(args.filepath)

    ret = transformer.get_transform("/perception/object_recognition/detection/objects", "map")
