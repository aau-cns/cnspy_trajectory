#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2024, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# BASED ON: https://github.com/aau-cns/cnspy_rosbag2csv
# just install "pip install cnspy-rosbag2csv"
########################################################################################################################
import yaml
from tqdm import tqdm
import numpy as np
from spatialmath import UnitQuaternion, SO3, SE3
from spatialmath.base.quaternions import qslerp
import geometry_msgs.msg
from cnspy_trajectory.HistoryBuffer import HistoryBuffer, get_key_from_value


class ROSBag_Pose:

    @staticmethod
    def get_pose(hist_poses: HistoryBuffer, timestamp) -> SE3:
        if hist_poses.exists_at_t(timestamp) is None:
            interpol = True
            # interpolate between poses
            [t1, T_GLOBAL_BODY_T1] = hist_poses.get_before_t(timestamp)
            [t2, T_GLOBAL_BODY_T2] = hist_poses.get_after_t(timestamp)

            if t1 is None or t2 is None:
                return None

            dt = t2 - t1
            dt_i = timestamp - t1
            i = dt_i / dt

            q0 = UnitQuaternion(T_GLOBAL_BODY_T1.R)
            q1 = UnitQuaternion(T_GLOBAL_BODY_T2.R)
            p0 = T_GLOBAL_BODY_T1.t
            p1 = T_GLOBAL_BODY_T2.t

            qr = UnitQuaternion(qslerp(q0.vec, q1.vec, i, shortest=True), norm=True)
            pr = p0 * (1 - i) + i * p1

            # interpolate between poses:
            T_GLOBAL_BODY = SE3.Rt(qr.R, pr, check=True)
            if not SE3.isvalid(T_GLOBAL_BODY, check=True):
                if T_GLOBAL_BODY.A is None:
                    return None
                else:
                    q = UnitQuaternion(SO3(T_GLOBAL_BODY.R, check=False), norm=True).unit()
                    T_GLOBAL_BODY = SE3.Rt(q.R, T_GLOBAL_BODY.t, check=True)
            return T_GLOBAL_BODY
        else:
            return hist_poses.get_at_t(timestamp)
        pass

    @staticmethod
    def extract_pose(bag, num_messages, topic_pose_body, round_decimals=6, T_BODY_SENSOR=None) -> HistoryBuffer:
        """
        Parameters
        ----------
        bag: file handle to the rosbag
        num_messages: number of messages contained in the bag file
        topic_pose_body: topic name of body pose
        round_decimals (optional): integer > 1
        T_BODY_SENSOR(optional): SE3 pose from Body to Sensor

        Returns HistoryBuffer
        -------
        """
        if round_decimals < 1:
            round_decimals = 1

        hist_poses = dict()
        ## extract the desired topics from the BAG file
        try:  # else already exists
            print("ROSBag_Pose.extract(): extracting pose...")
            cnt_poses = 0
            for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic == topic_pose_body:
                    T_GLOBAL_BODY = None
                    if hasattr(msg, 'header') and hasattr(msg, 'pose') and hasattr(msg, 'twist'):  # nav_msgs/Odometry Message
                        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                        q_GB = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z]

                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED || POSEWITHCOVARIANCE_STAMPED
                        if hasattr(msg.pose, 'covariance'):  # PoseWithCovariance
                            t = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                            q_GB = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z]
                        elif hasattr(msg.pose, 'position'):  # Pose
                            t = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                            q_GB = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                    msg.pose.orientation.z]

                        q = UnitQuaternion(q_GB, norm=True)
                        T_GLOBAL_BODY = SE3.Rt(q.R, t, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED
                        t = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                        q_GB = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z]

                        q = UnitQuaternion(q_GB, norm=True)
                        T_GLOBAL_BODY = SE3.Rt(q.R, t, check=True)
                        pass

                    elif hasattr(msg, 'header') and hasattr(msg, 'transform'):
                        t = np.array(
                            [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
                        q_GB = [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
                                msg.transform.rotation.z]
                        q = UnitQuaternion(q_GB, norm=True)
                        T_GLOBAL_BODY = SE3.Rt(q.R, t, check=True)
                    else:
                        print("\nROSBag_Pose.extract(): unsupported message " + str(msg))
                        continue

                    if T_GLOBAL_BODY is not None:
                        timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                        if T_BODY_SENSOR and isinstance(T_BODY_SENSOR, SE3):
                            hist_poses[timestamp] = T_GLOBAL_BODY * T_BODY_SENSOR
                        else:
                            hist_poses[timestamp] = T_GLOBAL_BODY
                        cnt_poses = cnt_poses + 1
                        pass
                pass

            if cnt_poses == 0:
                print("\nROSBag_Pose.extract(): no poses obtained!")
                return HistoryBuffer()
            else:
                print("\nROSBag_Pose.extract(): poses extracted: " + str(cnt_poses))
                pass
        except AssertionError as error:
            print(error)
            print(
                "ROSBag_Pose.extract(): Unexpected error while reading the bag file!\n * try: $ rosbag fix <bagfile> <fixed>")
            return HistoryBuffer()
        return HistoryBuffer(dict_t=hist_poses)

    @staticmethod
    def geometry_msgs_pose_to_SE3(pose):

        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        q_GB = [pose.orientation.w, pose.orientation.x, pose.orientation.y,
                pose.orientation.z]

        q = UnitQuaternion(q_GB).unit()
        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
        return T_GLOBAL_BODY

    @staticmethod
    def extract_poses(bag, dict_topic_pose_body, dict_senor_topic_pose, num_messages=None, round_decimals=6, dict_T_BODY_SENSOR=None) -> dict:
        """
        Parameters
        ----------
        bag: file handle to the rosbag
        num_messages: number of messages contained in the bag file
        dict_topic_pose_body: topic names of Body poses
        dict_senor_topic_pose: topic names for the final Sensor pose history
        round_decimals (optional): integer > 1
        dict_T_BODY_SENSOR (optional): SE3 pose from Body to Sensor: T_GLOBAL_SENSOR(t) = T_GLOBAL_BODY(t) * T_BODY_SENSOR

        Returns  a dictionary of HistoryBuffer: dict<senor_topic, HistoryBuffer<T_GLOBAL_SENSOR(t)>>
        -------
        """
        if num_messages is None:
            info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)
            num_messages = info_dict['messages']
        if round_decimals < 1:
            round_decimals = 1


        dict_hist_poses = dict()  # map<topic<timestamp, SE3>>
        for id_, topic_ in dict_senor_topic_pose.items():
            dict_hist_poses[topic_] = dict()

        try:  # else already exists
            print("ROSBag_Pose: extracting poses...")
            cnt_poses = 0
            for topic_, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic_ in dict_topic_pose_body.values():

                    id_topic = get_key_from_value(dict_topic_pose_body, topic_)
                    topic_sensor = dict_senor_topic_pose[id_topic]
                    T_GLOBAL_BODY = None
                    if hasattr(msg, 'header') and hasattr(msg, 'pose') and hasattr(msg, 'twist'):  # nav_msgs/Odometry Message
                        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                        q_GB = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z]

                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED
                        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                        q_GB = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z]

                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'transform'):
                        p = np.array(
                            [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
                        q_GB = [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
                                msg.transform.rotation.z]
                        q = UnitQuaternion(q_GB).unit()
                        T_GLOBAL_BODY = SE3.Rt(q.R, p, check=True)
                    else:
                        print("\nROSBag_Pose: unsupported message " + str(msg))
                        continue

                    if T_GLOBAL_BODY is not None:
                        timestamp = round(msg.header.stamp.to_sec(), round_decimals)

                        if dict_T_BODY_SENSOR and topic_sensor in dict_T_BODY_SENSOR.keys():
                            T_BODY_SENSOR = dict_T_BODY_SENSOR[topic_sensor]
                            dict_hist_poses[topic_sensor][timestamp] = T_GLOBAL_BODY * T_BODY_SENSOR
                        else:
                            dict_hist_poses[topic_sensor][timestamp] = T_GLOBAL_BODY
                        cnt_poses = cnt_poses + 1
                        pass
                pass

            if cnt_poses == 0:
                print("\nROSBag_Pose: no poses obtained!")
                return dict()
            else:
                print("\nROSBag_Pose: poses extracted: " + str(cnt_poses))

        except AssertionError as error:
            print(error)
            print(
                "ROSBag_Pose: Unexpected error while reading the bag file!\n * try: $ rosbag fix <bagfile> <fixed>")
            return dict()

        ## convert poses to History only if poses exist in bag file.
        dict_history = dict() # map<topic, history>
        for topic_sensor, hist_poses in dict_hist_poses.items():
            if len(hist_poses):
                dict_history[topic_sensor] = HistoryBuffer(dict_t=hist_poses)

        return dict_history
