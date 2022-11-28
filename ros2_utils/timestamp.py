from typing import Dict, List, Union

import matplotlib.pyplot as plt
import numpy as np
from builtin_interfaces.msg import Time

from ros2_utils.rosbag import BagFileParser


def ros2unix(ros_time: Time) -> np.uint64:
    """Convert ROS timestamp to unix time.
    Args:
        ros_time (Time)
    Returns:
        np.uint64
    """
    return np.uint64(ros_time.sec + ros_time.nanosec * 1e-9)


def utc2unix(utc_time: float) -> np.uint64:
    """Convert UTC timestamp to UNIX timestamp.
    Args:
        utc_time (float)
    Returns:
        np.uint64
    """
    return np.uint64(utc_time / 86400) + 25569


class TimestampParser:
    """Parse timestamp information from rosbag."""

    def __init__(self, bag_file: str) -> None:
        """
        Args:
            bag_file (str): Rosbag file path.
        """
        self.parser = BagFileParser(bag_file)

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
            data = self.parser.get_messages(name)
            real_stamp: np.ndarray = np.array(
                [stamp for stamp, _ in data],
                dtype=np.uint64,
            )
            header_stamp: np.ndarray = np.array(
                [ros2unix(msg.header.stamp) for _, msg in data],
                dtype=np.uint64,
            )
            ret[name] = np.stack([header_stamp, real_stamp])
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

    def plot_delay(self, topic_names: Union[str, List[str]]) -> None:
        """Plot delay between header timestamp and real timestamp.
        Args:
            topic_names (Union[str, List[str]]): Name of target topic.
        """
        _, ax = plt.subplots()
        stamps: Dict[str, np.ndarray] = self.get_stamp(topic_names)
        for name, stamp in stamps.items():
            x: np.ndarray = stamp[:, 0] / 1e9
            y: np.ndarray = (stamp[:, 1] - stamp[:, 0]) / 1e9
            ax.plot(x, y, "+", label=name)
        ax.set_ylabel("delay [s]")
        ax.set_xlabel("timestamp [s]")
        ax.legend()
        plt.show()
        plt.close()

    def plot_interval(self, topic_names: Union[str, List[str]]) -> None:
        """Plot interval.
        Args:
            topic_names (Union[str, List[str]]: Name of target topic.
        """
        _, ax = plt.subplots()
        stamps: Dict[str, np.ndarray] = self.get_stamp(topic_names)
        for name, stamp in stamps.items():
            x: np.ndarray = stamp[:, 0] / 1e9
            y: np.ndarray = np.diff(stamp[:, 0]) / 1e9
            y = np.insert(y, 0, axis=0)
            ax.plot(x, y, "+", label=name)
        ax.set_ylabel("interval [s]")
        ax.set_xlabel("timestamp [s]")
        ax.legend()
        plt.show()
        plt.close()
