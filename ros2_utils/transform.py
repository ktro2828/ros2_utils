from __future__ import annotations

from typing import Any, List, Optional, Tuple

import rclpy
import tf2_ros
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from ros2_utils.rosbag import BagFileParser


class TransformInterface(Node):
    """Parse bag file and transform recognized objects from base_link to map coordinate system."""

    def __init__(self, parser: BagFileParser, cache_time: float = 1000) -> None:
        """
        Args:
            parser (str): BagFileParser.
            cache_time (float): Transform buffer cache time[s]. Defaults to 1000.
        """
        super().__init__("transform_interface")
        self.parser: BagFileParser = parser

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=cache_time))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.set_transform()

    @classmethod
    def from_bag(cls, bag_file: str, cache_time: float = 1000) -> TransformInterface:
        """
        Args:
            bag_file (str): Bag file path (.db3).
            cache_time (float): Transform buffer cache time[s]. Defaults to 1000.
        """
        parser = BagFileParser(bag_file)
        return cls(parser, cache_time)

    def __del__(self) -> None:
        del self.parser

    def set_transform(self):
        tf_msgs: List[Tuple[int, TFMessage]] = self.parser.get_msg("/tf")
        for _, msg in tf_msgs:
            for trans in msg.transforms:
                self.tf_buffer.set_transform(trans, "default_authority")
        tf_static_msgs: List[Tuple[int, TFMessage]] = self.parser.get_msg("/tf_static")
        for _, msg in tf_static_msgs:
            for trans in msg.transforms:
                self.tf_buffer.set_transform_static(trans, "default_authority")

    def get_transform(
        self,
        topic_name: str,
        target_frame: str,
        duration: float = 0.0,
    ) -> List[Tuple[Optional[TransformStamped], Any]]:
        """
        Args:
            topic_name (str): Name of topic.
            target_frame (str): Name of target frame.
        Returns:
            ret (List[Tuple[Optional[TransformStamped], Any]]): List of transforms and messages.
                If lookup_transform() is failed, (None, message) will be set.
        """
        target_msgs: List[Tuple[int, Any]] = self.parser.get_msg(topic_name)
        ret: List[Tuple[Optional[TransformStamped], Any]] = []
        for _, msg in target_msgs:
            try:
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    Duration(seconds=duration),
                )
                ret.append((trans, msg))
            except Exception as e:
                print(e)
                ret.append((None, msg))

        return ret

    def get_transform_from_stamp(
        self,
        stamp: Time,
        target_frame: str,
        source_frame: str,
        duration: float = 0.0,
    ) -> Optional[TransformStamped]:
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                Duration(seconds=duration),
            )
            return trans
        except Exception as e:
            print(e)
            return None


if __name__ == "__main__":
    import argparse
    from tf2_geometry_msgs import do_transform_pose

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-f", "--filepath", type=str, help="Bag file path(.db3)")
    args = arg_parser.parse_args()

    rclpy.init()
    transformer = TransformInterface.from_bag(args.filepath)

    ret = transformer.get_transform("/perception/object_recognition/detection/objects", "map")
    for trans, msg in ret:
        if trans is None:
            continue
        for obj in msg.objects:
            transformed_pose = do_transform_pose(obj.kinematics.initial_pose_with_covariance.pose, trans)
