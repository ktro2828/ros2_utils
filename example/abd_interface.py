from __future__ import annotations

from typing import Any, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from abd_robot_interface_msgs.msg import CMABD01, CMABD02, CMABD03, CMABD04, CMABD05, CMABD06, CMABD07
from geometry_msgs.msg import Pose
from matplotlib.pyplot import Axes
from pyquaternion import Quaternion
from geometry_msgs.msg import TransformStamped
from tqdm import tqdm

from ros2_utils.rosbag import BagFileParser
from ros2_utils.tf2_geometry_msgs import do_transform_pose
from ros2_utils.timestamp import TimestampParser, ros2unix
from ros2_utils.transform import TransformInterface
from perception_eval.tool import Gmm


class ABDRobotInterface:
    JARI_X = 16677.0494
    JARI_Y = 92991.4717
    JARI_Z = 24.7107
    JARI_YAW = 1.4756
    TARGET_LENGTH = 4.023
    TARGET_WIDTH = 1.712
    TARGET_HEIGHT = 1.427

    def __init__(self, parser: BagFileParser, cache_time: float = 1000) -> None:
        self.parser = parser
        self.tf = TransformInterface(parser, cache_time=cache_time)

        self.abd01: List[Tuple[int, CMABD01]] = []
        self.abd02: List[Tuple[int, CMABD02]] = []
        self.abd03: List[Tuple[int, CMABD03]] = []
        self.abd04: List[Tuple[int, CMABD04]] = []
        self.abd05: List[Tuple[int, CMABD05]] = []
        self.abd06: List[Tuple[int, CMABD06]] = []
        self.abd07: List[Tuple[int, CMABD07]] = []

        self.__parse_can_msg()

    @classmethod
    def from_bag(cls, bag_file: str, cache_time: float = 1000) -> ABDRobotInterface:
        parser = BagFileParser(bag_file)
        return cls(parser, cache_time)

    def __parse_can_msg(self) -> None:
        """Parse abd_robot_interface_msgs from /abd_robot/from_can_bus."""
        frame_id: str = "base_link"
        msgs = self.parser.get_msg("/abd_robot/from_can_bus")
        for stamp, msg in tqdm(msgs, "Parsing /abd_robot/from_can_bus"):
            msg_id = msg.id
            if msg_id == 0x640:
                abd01_msg = CMABD01()
                abd01_msg.header.stamp = msg.header.stamp
                abd01_msg.header.frame_id = frame_id
                abd01_msg.refxpos = (
                    np.int32((msg.data[3] << 24) + (msg.data[2] << 16) + (msg.data[1] << 8) + msg.data[0]) * 0.001
                )
                abd01_msg.refypos = (
                    np.int32((msg.data[7] << 24) + (msg.data[6] << 16) + (msg.data[5] << 8) + (msg.data[4])) * 0.001
                )
                self.abd01.append((stamp, abd01_msg))
            elif msg_id == 0x641:
                abd02_msg = CMABD02()
                abd02_msg.header.stamp = msg.header.stamp
                abd02_msg.header.frame_id = frame_id
                abd02_msg.vx = np.int32((msg.data[1] << 8) + (msg.data[0])) * 0.01
                abd02_msg.vy = np.int32((msg.data[3] << 8) + (msg.data[2])) * 0.01
                abd02_msg.vz = np.int32((msg.data[5] << 8) + (msg.data[4])) * 0.01
                abd02_msg.ax = np.int32((msg.data[7] << 8) + (msg.data[6])) * 0.01
                self.abd02.append((stamp, abd02_msg))
            elif msg_id == 0x642:
                abd03_msg = CMABD03()
                abd03_msg.header.stamp = msg.header.stamp
                abd03_msg.header.frame_id = frame_id
                abd03_msg.yaw = (
                    np.int32((msg.data[3] << 24) + (msg.data[2] << 16) + (msg.data[1] << 8) + msg.data[0]) * 0.01
                )
                abd03_msg.mptime = (
                    np.int32((msg.data[7] << 24) + (msg.data[6] << 16) + (msg.data[5] << 8) + (msg.data[4])) * 0.001
                )
                self.abd03.append((stamp, abd03_msg))
            elif msg_id == 0x643:
                abd04_msg = CMABD04()
                abd04_msg.header.stamp = msg.header.stamp
                abd04_msg.header.frame_id = frame_id
                abd04_msg.rftargetx = (
                    np.int32((msg.data[3] << 24) + (msg.data[2] << 16) + (msg.data[1] << 8) + (msg.data[0])) * 0.001
                )
                abd04_msg.rftargety = (
                    np.int32((msg.data[7] << 24) + (msg.data[6] << 16) + (msg.data[5] << 8) + (msg.data[4])) * 0.001
                )
                self.abd04.append((stamp, abd04_msg))
            elif msg_id == 0x644:
                abd05_msg = CMABD05()
                abd05_msg.header.stamp = msg.header.stamp
                abd05_msg.header.frame_id = frame_id
                abd05_msg.longrange = np.int16((msg.data[1] << 8) + (msg.data[0])) * 0.01
                abd05_msg.latrange = np.int16((msg.data[3] << 8) + (msg.data[2])) * 0.01
                abd05_msg.longvel = np.int16((msg.data[5] << 8) + (msg.data[4])) * 0.01
                abd05_msg.latvel = np.int16((msg.data[7] << 8) + (msg.data[6])) * 0.01
                self.abd05.append((stamp, abd05_msg))
            elif msg_id == 0x645:
                abd06_msg = CMABD06()
                abd06_msg.header.stamp = msg.header.stamp
                abd06_msg.header.frame_id = frame_id
                abd06_msg.othvfwdv = np.int16((msg.data[1] << 8) + (msg.data[0])) * 0.01
                abd06_msg.othvlatv = np.int16((msg.data[3] << 8) + (msg.data[2])) * 0.01
                abd06_msg.othvfwda = np.int16((msg.data[5] << 8) + (msg.data[4])) * 0.01
                abd06_msg.othvlata = np.int16((msg.data[7] << 8) + (msg.data[6])) * 0.01
                self.abd06.append((stamp, abd06_msg))
            elif msg_id == 0x646:
                abd07_msg = CMABD07()
                abd07_msg.header.stamp = msg.header.stamp
                abd07_msg.header.frame_id = frame_id
                abd07_msg.othvehyaw = (
                    np.int32((msg.data[3] << 24) + (msg.data[2] << 16) + (msg.data[1] << 8) + (msg.data[0])) * 0.01
                )
                abd07_msg.othvehpit = np.int16((msg.data[5] << 8) + (msg.data[4])) * 0.01
                abd07_msg.othvlater = np.int16((msg.data[7] << 8) + (msg.data[6])) * 0.001
                self.abd07.append((stamp, abd07_msg))

    def get_target_pose(self, frame_id: str) -> np.ndarray:
        """Get center position of target w.r.t base_link.
        Returns:
            numpy.ndarray: (stamp, Pose), In shape (N, 2).
        """
        ret = []
        for stamp, abd04_msg in tqdm(self.abd04, "Loading target vehicle position from CMABD04"):
            trans = self.tf.get_transform_from_stamp(
                abd04_msg.header.stamp,
                target_frame="base_link",
                source_frame="map",
            )
            if trans is None:
                continue
            abd07_msg: CMABD07 = self.get_current_msg(stamp, self.abd07)[0]

            # m_H_w
            m_H_w = np.eye(3)
            m_H_w[:2, :2] = self.get_rotation_mat(self.JARI_YAW)
            m_H_w[:2, 2] = (self.JARI_X, self.JARI_Y)

            # w_H_c
            w_H_c = np.eye(3)
            w_H_c[:2, :2] = self.get_rotation_mat(abd07_msg.othvehyaw, deg2rad=True)
            w_H_c[:2, 2] = (abd04_msg.rftargetx, abd04_msg.rftargety)

            # c_H_d
            c_H_d = np.eye(3)
            c_H_d[:2, :2] = self.get_rotation_mat(0)
            c_H_d[:2, 2] = (self.TARGET_LENGTH / 2.0, 0.0)

            m_H_c = np.matmul(m_H_w, w_H_c)
            m_H_d = np.matmul(m_H_c, c_H_d)

            m_R_d = np.eye(3)
            m_R_d[:2, :2] = m_H_d[:2, :2]
            m_Q_d = Quaternion(matrix=m_R_d)

            # Pose w.r.t map
            m_P_d = Pose()
            m_P_d.position.x = m_H_d[0, 2]
            m_P_d.position.y = m_H_d[1, 2]
            m_P_d.position.z = self.JARI_Z + (self.TARGET_HEIGHT / 2.0)
            m_P_d.orientation.w = m_Q_d.w
            m_P_d.orientation.x = m_Q_d.x
            m_P_d.orientation.y = m_Q_d.y
            m_P_d.orientation.z = m_Q_d.z

            if frame_id == "base_link":
                # Pose w.r.t base_link
                b_P_d: Pose = do_transform_pose(m_P_d, trans)
                ret.append([stamp, b_P_d])
            elif frame_id == "map":
                ret.append([stamp, m_P_d])
            else:
                raise ValueError(f"Unexpected frame_id: {frame_id}")
        return np.array(ret)

    def get_target_velocity(self, frame_id: str) -> np.ndarray:
        """Returns target velocity w.r.t base_link"""
        ret = []
        for stamp, abd06_msg in tqdm(self.abd06, "Loading target vehicle velocity from CMABD06"):
            abd04_msg: CMABD04 = self.get_current_msg(stamp, self.abd04)[0]
            abd07_msg: CMABD07 = self.get_current_msg(stamp, self.abd07)[0]
            # m_H_w
            m_H_w = np.eye(3)
            m_H_w[:2, :2] = self.get_rotation_mat(self.JARI_YAW)
            m_H_w[:2, 2] = (self.JARI_X, self.JARI_Y)

            # w_H_c
            w_H_c = np.eye(3)
            w_H_c[:2, :2] = self.get_rotation_mat(abd07_msg.othvehyaw, deg2rad=True)
            w_H_c[:2, 2] = (abd04_msg.rftargetx, abd04_msg.rftargety)

            # c_H_d
            c_H_d = np.eye(3)
            c_H_d[:2, :2] = self.get_rotation_mat(0)
            c_H_d[:2, 2] = (self.TARGET_LENGTH / 2.0, 0.0)

            m_H_c = np.matmul(m_H_w, w_H_c)
            m_H_d = np.matmul(m_H_c, c_H_d)

            w_v_c = np.array([abd06_msg.othvfwdv, abd06_msg.othvlatv])
            m_V_d_ = np.matmul(m_H_d[:2, :2], np.ones(2))
            m_V_d = np.linalg.norm(w_v_c) * m_V_d_ / np.linalg.norm(m_V_d_)

            if frame_id == "base_link":
                trans: Optional[TransformStamped] = self.tf.get_transform_from_stamp(
                    abd06_msg.header.stamp,
                    target_frame="base_link",
                    source_frame="map",
                )
                if trans is None:
                    continue
                rotation_matrix = Quaternion(
                    trans.transform.rotation.w,
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                ).rotation_matrix
                v = np.matmul(rotation_matrix, np.array([1, 1, 0]))[:2]
                vx, vy = np.linalg.norm(m_V_d) * v / np.linalg.norm(v)
                ret.append([stamp, vx, vy])
            elif frame_id == "map":
                ret.append([stamp, m_V_d[0], m_V_d[1]])
            else:
                raise ValueError(f"Unexpected frame_id: {frame_id}")
        return np.array(ret)

    def get_bottom_pos(self, frame_id: str = "base_link") -> np.ndarray:
        """Get bottom center position of target w.r.t base_link.
        Returns:
            numpy.ndarray: (stamp, Pose), In shape (N, 2).
        """
        ret = []
        poses = self.get_target_pose(frame_id=frame_id)
        for stamp, pose in poses:
            yaw = Quaternion(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ).yaw_pitch_roll[0]
            bottom_x = pose.position.x - (self.TARGET_LENGTH / 2.0) * np.cos(yaw)
            bottom_y = pose.position.y - (self.TARGET_LENGTH / 2.0) * np.sin(yaw)
            ret.append([stamp, bottom_x, bottom_y])

        return np.array(ret)

    @staticmethod
    def get_current_msg(stamp: int, msg: Tuple[int, Any]) -> Any:
        msg_arr = np.array(msg)
        idx = np.argmin(np.abs(msg_arr[:, 0] - stamp))
        return msg_arr[:, 1:][idx]

    @staticmethod
    def get_rotation_mat(theta: float, deg2rad: bool = False) -> np.ndarray:
        if deg2rad:
            theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    def get_scan_delay(self, scan_phase: float, deg2rad: bool) -> np.ndarray:
        """
        Args:
            scan_phase (float): [0, 2*PI]
            deg2rad (bool): If True, convert scan_phase degree to radian.
        Returns:
            numpy.ndarray: [stamp, delay], In shape (N, 2).
        """
        ret = []
        bottom_pos = self.get_bottom_pos()
        if deg2rad:
            scan_phase = np.deg2rad(scan_phase)

        for stamp, x, y in bottom_pos:
            yaw = np.arctan2(y, x)  # [-pi, pi]
            if yaw > scan_phase:
                yaw = 2 * np.pi - (yaw - scan_phase)
            else:
                yaw = scan_phase - yaw
            delay = 0.1 * yaw / (2 * np.pi)
            ret.append([stamp, delay])
        return np.array(ret)

    def plot_scan_delay(self, scan_phase: float, deg2rad: bool, ax: Optional[Axes] = None) -> Axes:
        if ax is None:
            _, ax = plt.subplots()

        delays = self.get_scan_delay(scan_phase, deg2rad)
        ax.plot(delays[:, 0] / 1e9, delays[:, 1], "+", label="scan delay")
        print(delays[:, 1].mean())
        ax.legend()

        return ax

    def plot_position(
        self,
        frame_id: str,
        poses: Optional[np.ndarray] = None,
        ax1: Optional[Axes] = None,
        ax2: Optional[Axes] = None,
    ) -> Tuple[Axes, Axes]:
        """Plot positions."""
        if poses is None:
            poses = self.get_target_pose(frame_id=frame_id)

        fig = plt.figure()
        ax1 = fig.add_subplot(1, 2, 1)
        ax2 = fig.add_subplot(1, 2, 2)
        positions = np.array([[p.position.x, p.position.y] for p in poses[:, 1]])
        ax1.plot(poses[:, 0] / 1e9, positions[:, 0], label=f"x@{frame_id}", linestyle="solid")
        ax2.plot(poses[:, 0] / 1e9, positions[:, 1], label=f"y@{frame_id}", linestyle="solid")
        ax1.legend()
        ax2.legend()

        return ax1, ax2

    def plot_velocity(
        self,
        frame_id: str,
        velocities: Optional[np.ndarray] = None,
        ax1: Optional[Axes] = None,
        ax2: Optional[Axes] = None,
    ) -> Tuple[Axes, Axes]:
        """Plot velocities."""
        if velocities is None:
            velocities = self.get_target_velocity(frame_id=frame_id)

        fig = plt.figure()
        ax1 = fig.add_subplot(1, 2, 1)
        ax2 = fig.add_subplot(1, 2, 2)
        ax1.plot(velocities[:, 0] / 1e9, velocities[:, 1], label=f"vx@{frame_id}")
        ax2.plot(velocities[:, 0] / 1e9, velocities[:, 2], label=f"vy@{frame_id}")
        ax1.legend()
        ax2.legend()

        return ax1, ax2


def main():
    bag_file: str = "/media/kotarouetake/Volume/DATASET/tier4/jari/2022-10-04_cutin_name_change/rosbag/b64370f4-8b54-4cdd-863d-11d942f6bc7f_2022-10-04-11-04-09/b64370f4-8b54-4cdd-863d-11d942f6bc7f_2022-10-04-11-04-09_0.db3"
    rclpy.init()
    abd_interface = ABDRobotInterface.from_bag(bag_file)
    # stamp_parser = TimestampParser(bag_file)
    # ax = stamp_parser.plot_delay(
    #     topic_names=[
    #         "/sensing/lidar/top/velodyne_packets",
    #         "/perception/object_recognition/objects",
    #     ]
    # )
    # ax = abd_interface.plot_scan_delay(150, True, ax)
    # plt.show()
    # abd_interface.plot_velocity("base_link")
    # abd_interface.plot_position("map")

    # detected_objects_data = abd_interface.parser.get_msg("/perception/object_recognition/detection/objects")
    # data = np.array(
    #     [
    #         [
    #             stamp,
    #             obj.kinematics.pose_with_covariance.pose.position.x,
    #             obj.kinematics.pose_with_covariance.pose.position.y,
    #         ]
    #         for stamp, msg in detected_objects_data
    #         for obj in msg.objects
    #     ]
    # )
    # _, ax = plt.subplots()
    # ax.plot(data[:, 0] / 1e9, data[:, 1], label="x")
    # plt.show()

    # Create sample for GMM
    poses = abd_interface.get_target_pose("base_link")
    velocities = abd_interface.get_target_velocity("map")
    sample = []
    gt_pos = []
    gt_vel = []
    for stamp, pose in poses:
        vel = abd_interface.get_current_msg(stamp, velocities)
        sample.append(
            [pose.position.x, pose.position.y, abd_interface.TARGET_WIDTH, abd_interface.TARGET_HEIGHT, vel[0], vel[1]]
        )
        gt_pos.append([stamp, pose.position.x, pose.position.y])
        gt_vel.append([stamp, vel[0], vel[1]])
    sample = np.array(sample)  # (N, 6)
    gt_pos = np.array(gt_pos)  # (N, 3)
    gt_vel = np.array(gt_vel)  # (N, 3)

    gmm_pos = Gmm.load("/media/kotarouetake/Volume/DATASET/tier4/jari/model_params/gmm_xywlvxvy_xy.sav")
    gmm_vel = Gmm.load("/media/kotarouetake/Volume/DATASET/tier4/jari/model_params/gmm_xywlvxvy_vxvy.sav")

    est_pos_err = gmm_pos.predict(sample)  # (N, 2)
    est_vel_err = gmm_vel.predict(sample)  # (N, 2)
    est_pos = gt_pos[:, 1:] + est_pos_err
    est_vel = gt_vel[:, 1:] + est_vel_err

    pos_ax1, pos_ax2 = abd_interface.plot_position("base_link")

    pos_ax1.plot(gt_pos[:, 0] / 1e9, est_pos[:, 0], label="GT+GMM_x", linestyle="dashed")
    pos_ax2.plot(gt_pos[:, 0] / 1e9, est_pos[:, 1], label="GT+GMM_y", linestyle="dashed")
    pos_ax1.legend()
    pos_ax2.legend()
    plt.show()

    vel_ax1, vel_ax2 = abd_interface.plot_velocity("map")
    vel_ax1.plot(gt_vel[:, 0] / 1e9, est_vel[:, 0], label="GT+GMM_vx", linestyle="dashed")
    vel_ax2.plot(gt_vel[:, 0] / 1e9, est_vel[:, 1], label="GT+GMM_vy", linestyle="dashed")
    vel_ax1.legend()
    vel_ax2.legend()
    plt.show()


if __name__ == "__main__":
    main()
