import argparse
import sqlite3
from typing import Any, Dict, List, Tuple

import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from ros2_utils.utils import class2dict


class BagFileParser:
    """Parse bag file."""

    def __init__(self, bag_file: str) -> None:
        """
        Args:
            bag_file (str): Bag file path (.db3).
            topic_file (str): Target topic file path (.yaml)
        """
        self.connect = sqlite3.connect(bag_file)
        self.cursor = self.connect.cursor()

        # create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type from topics").fetchall()
        self.topic_type = {name_of: type_of for _, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, _ in topics_data}
        self.topic_msg = {name_of: get_message(type_of) for _, name_of, type_of in topics_data}

    def __del__(self) -> None:
        self.connect.close()

    def get_msg(self, topic_name: str) -> List[Tuple[int, Any]]:
        """Returns the list of timestamps and messages.
        Args:
            topic_name (str): Target topic name.

        Returns:
            List[Tuple[int, Any]]: [(timestamp0, message0), (timestamp1, message1), ...]
        """
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}").fetchall()
        # Deserialize all and timestamp them
        return [(timestamp, deserialize_message(data, self.topic_msg[topic_name])) for timestamp, data in rows]

    def to_df(self, topic_name: str) -> pd.DataFrame:
        msgs = self.get_msg(topic_name)
        data_list: List[Dict[str, Any]] = []
        for _, msg in msgs:
            data_list.append(class2dict(msg))


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-f", "--filepath", type=str, help="Bag file path(.db3)")
    # arg_parser.add_argument("-t", "--topics", type=str, nargs="+", help="topic1 topic2 ...")
    args = arg_parser.parse_args()

    parser = BagFileParser(args.filepath)
    parser.get_msg("/abd_robot/")
