#!/usr/bin/env python

from copy import deepcopy
import geometry_msgs.msg
import numpy as np
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import PyKDL
import tf2_geometry_msgs
import tf2_ros

from tf.transformations import quaternion_from_matrix as matrix2quaternion
from tf.transformations import unit_vector as normalize_vector
from tf.transformations import vector_norm


def outer_product_matrix(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def cross_product(a, b):
    return np.dot(outer_product_matrix(a), b)


def rotation_matrix_from_axis(
        first_axis=(1, 0, 0), second_axis=(0, 1, 0), axes='xy'):
    if axes not in ['xy', 'yx', 'xz', 'zx', 'yz', 'zy']:
        raise ValueError("Valid axes are 'xy', 'yx', 'xz', 'zx', 'yz', 'zy'.")
    e1 = normalize_vector(first_axis)
    e2 = normalize_vector(second_axis - np.dot(second_axis, e1) * e1)
    if axes in ['xy', 'zx', 'yz']:
        third_axis = cross_product(e1, e2)
    else:
        third_axis = cross_product(e2, e1)
    e3 = normalize_vector(
        third_axis - np.dot(third_axis, e1) * e1 - np.dot(third_axis, e2) * e2)
    first_index = ord(axes[0]) - ord('x')
    second_index = ord(axes[1]) - ord('x')
    third_index = ((first_index + 1) ^ (second_index + 1)) - 1
    indices = [first_index, second_index, third_index]
    return np.vstack([e1, e2, e3])[np.argsort(indices)].T



class ArmPoseToBoxes(ConnectionBasedTransport):

    def __init__(self):
        super(ArmPoseToBoxes, self).__init__()
        self._duration_timeout = rospy.get_param("~timeout", 3.0)
	self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        
        self.pub = self.advertise('~output/boxes', BoundingBoxArray, queue_size=1)
        self.pose_pub = self.advertise('~output/poses',
                                       geometry_msgs.msg.PoseArray,
                                       queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PeoplePoseArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _convert(self, msg):
        header = deepcopy(msg.header)
        pose_array_msg = geometry_msgs.msg.PoseArray(header=header)
        boxes_msg = BoundingBoxArray(header=header)
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame_id,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rospy.Duration(self._duration_timeout))
            pykdl_transform_base_to_camera = tf2_geometry_msgs.transform_to_kdl(
                transform)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return

        left_elbow_name = "left elbow"
        right_elbow_name = "right elbow"
        left_wrist_name = "left wrist"
        right_wrist_name = "right wrist"
        for person in msg.poses:
            target_poses = []
            if left_elbow_name in person.limb_names and left_wrist_name in person.limb_names:
                elbow_index = person.limb_names.index(left_elbow_name)
                wrist_index = person.limb_names.index(left_wrist_name)
                scores = np.array(person.scores)
                rospy.loginfo("[left] {}".format(np.average(scores)))
                # elbow_score = person.scores[elbow_index]
                # wrist_score = person.scores[wrist_index]
                # rospy.loginfo("[left] elbow:{} wrist:{}".format(elbow_score, wrist_score))
                # if elbow_score > 0.6 and wrist_score > 0.6:
                if np.average(scores) > 0.45:
                    pose = person.poses[elbow_index]
                    elbow_pose = [pose.position.x,
                                  pose.position.y,
                                  pose.position.z]
                    pose = person.poses[wrist_index]
                    wrist_pose = [pose.position.x,
                                  pose.position.y,
                                  pose.position.z]
                    target_poses.append((elbow_pose, wrist_pose))

            if right_elbow_name in person.limb_names and right_wrist_name in person.limb_names:
                elbow_index = person.limb_names.index(right_elbow_name)
                wrist_index = person.limb_names.index(right_wrist_name)
                scores = np.array(person.scores)
                rospy.loginfo("[right] {}".format(np.average(scores)))
            #     # elbow_score = person.scores[elbow_index]
            #     # wrist_score = person.scores[wrist_index]
            #     # rospy.loginfo("[right] elbow:{} wrist:{}".format(elbow_score, wrist_score))
            #     # if elbow_score > 0.6 and wrist_score > 0.6:
                if np.average(scores) > 0.45:
                    pose = person.poses[elbow_index]
                    elbow_pose = [pose.position.x,
                                  pose.position.y,
                                  pose.position.z]
                    pose = person.poses[wrist_index]
                    wrist_pose = [pose.position.x,
                                  pose.position.y,
                                  pose.position.z]
                    target_poses.append((elbow_pose, wrist_pose))
                
            dim_x = 10.0
            for elbow_pose, wrist_pose in target_poses:
                wrist_pose = np.array(wrist_pose)
                elbow_pose = np.array(elbow_pose)
                v = wrist_pose - elbow_pose
                v_base = pykdl_transform_base_to_camera * PyKDL.Vector(*wrist_pose) \
                         - pykdl_transform_base_to_camera * PyKDL.Vector(*elbow_pose)
                v_base = np.array([v_base.x(), v_base.y(), v_base.z()])
                v_norm = np.linalg.norm(v_base)
                v_base = normalize_vector(v_base)

                rospy.loginfo("v_base:{}, {}, {}".format(*v_base))
                rospy.loginfo("norm:{}".format(v_norm))
                
                # vec_v_base = pykdl_transform_base_to_camera.M * PyKDL.Vector(*(wrist_pose - elbow_pose))
                # vec_v_base = np.array([vec_v_base.x(), vec_v_base.y(),vec_v_base.z()])
                # vec_v_base = normalize_vector(vec_v_base)
                # rospy.loginfo("vec_v_base {}, {}, {}".format(*vec_v_base))

                if abs(v_base[2]) > 0.7 and v_norm > 0.17:
                    continue
                matrix = np.eye(4)
                matrix[:3, :3] = rotation_matrix_from_axis(v, axes="xz")
                q_xyzw = matrix2quaternion(matrix)

                pose = geometry_msgs.msg.Pose()
                pose.position.x = wrist_pose[0]
                pose.position.y = wrist_pose[1]
                pose.position.z = wrist_pose[2]
                pose.orientation.x = q_xyzw[0]
                pose.orientation.y = q_xyzw[1]
                pose.orientation.z = q_xyzw[2]
                pose.orientation.w = q_xyzw[3]
                pose_array_msg.poses.append(pose)

                trans_vector = np.array([dim_x / 2.0, 0, 0, 1])
                trans = np.dot(matrix, trans_vector) + np.array(
                    [wrist_pose[0],
                     wrist_pose[1],
                     wrist_pose[2],
                     0.0])

                box_msg = BoundingBox(header=header)

                box_msg.pose.position.x = trans[0]
                box_msg.pose.position.y = trans[1]
                box_msg.pose.position.z = trans[2]
                box_msg.pose.orientation = pose.orientation
                box_msg.dimensions.x = dim_x
                box_msg.dimensions.y = 0.1
                box_msg.dimensions.z = 0.1
                boxes_msg.boxes.append(box_msg)

            self.pub.publish(boxes_msg)
            self.pose_pub.publish(pose_array_msg)
                

if __name__ == '__main__':
    rospy.init_node('arm_pose_to_boxes')
    ArmPoseToBoxes()
    rospy.spin()
