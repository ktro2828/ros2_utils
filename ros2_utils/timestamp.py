from __future__ import annotations

import math
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional, Union

import matplotlib.pyplot as plt
import numpy as np
from builtin_interfaces.msg import Time
from matplotlib.pyplot import Axes

from ros2_utils.rosbag import BagFileParser


def ros2unix(ros_time: Time) -> float:
    """Convert ROS timestamp to UNIX timestamp.
    Args:
        ros_time (Time)
    Returns:
        float
    """
    return ros_time.sec + ros_time.nanosec * 1e-9


def unix2ros(unix_time: float) -> Time:
    """Convert UNIX timestamp to ROS timestamp.
    Args:
        unix_time (float)
    Returns:
        ros_time (Time)
    """
    ros_time = Time()
    nanosec, sec = math.modf(unix_time)
    ros_time.sec = int(sec)
    ros_time.nanosec = int(nanosec * 1e9)
    return ros_time


def ros2datetime(ros_time: Time, **kwargs) -> datetime:
    """Convert ROS timestamp to datetime.
    Args:
        ros_time (Time)
    Returns:
        datetime
    """
    tz = timezone(timedelta(**kwargs))
    return datetime.fromtimestamp(ros2unix(ros_time), tz)


def utc2datetime(utc_time: float, **kwargs) -> datetime:
    """Convert UTC timestamp to datetime.
    Args:
        utc_time (float)
    Returns:
        datetime
    """
    tz = timezone(timedelta(**kwargs))
    return datetime.fromtimestamp(utc_time, tz)


class TimestampParser:
    """Parse timestamp information from rosbag."""

    def __init__(self, parser: BagFileParser) -> None:
        """
        Args:
            bag_file (str): Rosbag file path.
        """
        self.parser = parser

    @classmethod
    def from_bag(cls, bag_file: str) -> TimestampParser:
        parser = BagFileParser(bag_file)
        return cls(parser)

    def __del__(self) -> None:
        del self.parser

    def get_stamp(self, topic_names: Union[str, List[str]]) -> Dict[str, np.ndarray]:
        """Returns header timestamp and real timestamp.
        Args:
            topic_names (Union[str, List[str]]): Name of target topic.
        Returns:
            ret (Dict[str, np.ndarray]): In shape (N, 2).
                ret[:, 0] is header timestamp, ret[:, 1] is real timestamp.
        """
        if isinstance(topic_names, str):
            topic_names = [topic_names]

        ret: Dict[str, np.ndarray] = {}
        for name in topic_names:
            data = self.parser.get_msg(name)
            real_stamp: np.ndarray = np.array(
                [stamp / 1e9 for stamp, _ in data],
            )
            header_stamp: np.ndarray = np.array(
                [ros2unix(msg.header.stamp) for _, msg in data],
            )
            ret[name] = np.stack([header_stamp, real_stamp], axis=-1)
        return ret

    def get_delay(self, topic_names: Union[str, List[str]]) -> Dict[str, np.ndarray]:
        """Returns delay between header timestamp and real timestamp.
        Args:
            topic_names (Union[str, List[str]]): Name of target topic.
        Returns:
            ret (Dict[str, np.ndarray]): In shape (N,)
        """
        if isinstance(topic_names, str):
            topic_names = [topic_names]

        stamps: Dict[str, np.ndarray] = self.get_stamp(topic_names)
        ret: Dict[str, np.ndarray] = {}
        for name in topic_names:
            ret[name] = stamps[name][:, 1] - stamps[name][:, 0]
        return ret

    def get_interval(self, topic_names: Union[str, List[str]]) -> Dict[str, np.ndarray]:
        """Returns interval of topics.
        Args:
            topic_names (Union[str, List[str]])
        Returns:
            ret (Dict[str, np.ndarray]): In shape (N,)
        """
        if isinstance(topic_names, str):
            topic_names = [topic_names]
        stamps: Dict[str, np.ndarray] = self.get_stamp(topic_names)
        ret: Dict[str, np.ndarray] = {}
        for name in topic_names:
            ret[name] = np.diff(stamps[name][:, 0])
        return ret

    def plot_delay(
        self,
        topic_names: Union[str, List[str]],
        ax: Optional[Axes] = None,
    ) -> Axes:
        """Plot delay between header timestamp and real timestamp.
        Args:
            topic_names (Union[str, List[str]]): Name of target topic.
            ax (Optional[Axes]): matplotlib.Axes. Defaults to None.
        Returns:
            ax (Axes): matplotlib.Axes.
        """
        if ax is None:
            _, ax = plt.subplots()

        stamps: Dict[str, np.ndarray] = self.get_stamp(topic_names)
        for name, stamp in stamps.items():
            x: np.ndarray = stamp[:, 0]
            y: np.ndarray = stamp[:, 1] - stamp[:, 0]
            # Plot delay
            ax.plot(x, y, "+", label=name)
        ax.set_ylabel("delay [s]")
        ax.set_xlabel("timestamp [s]")
        ax.legend()
        return ax

    def plot_interval(self, topic_names: Union[str, List[str]], ax: Optional[Axes] = None) -> Axes:
        """Plot interval.
        Args:
            topic_names (Union[str, List[str]]: Name of target topic.
        """
        if ax is None:
            _, ax = plt.subplots()

        stamps: Dict[str, np.ndarray] = self.get_stamp(topic_names)
        for name, stamp in stamps.items():
            x: np.ndarray = stamp[:, 0]
            y: np.ndarray = np.diff(stamp[:, 0])
            y = np.insert(y, 0, axis=0)
            ax.plot(x, y, "+", label=name)
        ax.set_ylabel("interval [s]")
        ax.set_xlabel("timestamp [s]")
        ax.legend()
        return ax


if __name__ == "__main__":
    import argparse

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-f", "--filepath", type=str, help="Bag file path(.db3)")
    args = arg_parser.parse_args()

    ts_parser = TimestampParser.from_bag(args.filepath)

    ts_parser.plot_delay("/perception/object_recognition/objects")
    plt.show()
