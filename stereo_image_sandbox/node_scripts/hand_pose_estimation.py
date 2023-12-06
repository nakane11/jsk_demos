#!/usr/bin/env python3

import os
import sys

from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
import message_filters
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import Segment


# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    import cv2
    import mediapipe as mp
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA
    import cv2  # NOQA
    import mediapipe as mp
    sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA

# cv_bridge_python3 import
if os.environ['ROS_PYTHON_VERSION'] == '3':
    from cv_bridge import CvBridge
else:
    ws_python3_paths = [p for p in sys.path if 'devel/lib/python3' in p]
    if len(ws_python3_paths) == 0:
        # search cv_bridge in workspace and append
        ws_python2_paths = [
            p for p in sys.path if 'devel/lib/python2.7' in p]
        for ws_python2_path in ws_python2_paths:
            ws_python3_path = ws_python2_path.replace('python2.7', 'python3')
            if os.path.exists(os.path.join(ws_python3_path, 'cv_bridge')):
                ws_python3_paths.append(ws_python3_path)
        if len(ws_python3_paths) == 0:
            opt_python3_path = '/opt/ros/{}/lib/python3/dist-packages'.format(
                os.getenv('ROS_DISTRO'))
            sys.path = [opt_python3_path] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(opt_python3_path)
        else:
            sys.path = [ws_python3_paths[0]] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(ws_python3_paths[0])
    else:
        from cv_bridge import CvBridge


class HandPoseEstimation(ConnectionBasedTransport):

    def __init__(self):
        super(HandPoseEstimation, self).__init__()

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        self.INDEX2FINGERNAME = {
            self.mp_hands.HandLandmark.WRIST: "wrist",
            self.mp_hands.HandLandmark.THUMB_CMC: "thumb_mcp",
            self.mp_hands.HandLandmark.THUMB_MCP: "thumb_pip",
            self.mp_hands.HandLandmark.THUMB_IP: "thumb_dip",
            self.mp_hands.HandLandmark.THUMB_TIP: "thumb_tip",
            self.mp_hands.HandLandmark.INDEX_FINGER_MCP: "index_mcp",
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP: "index_pip",
            self.mp_hands.HandLandmark.INDEX_FINGER_DIP: "index_dip",
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP: "index_tip",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP: "middle_mcp",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP: "middle_pip",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_DIP: "middle_dip",
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP: "middle_tip",
            self.mp_hands.HandLandmark.RING_FINGER_MCP: "ring_mcp",
            self.mp_hands.HandLandmark.RING_FINGER_PIP: "ring_pip",
            self.mp_hands.HandLandmark.RING_FINGER_DIP: "ring_dip",
            self.mp_hands.HandLandmark.RING_FINGER_TIP: "ring_tip",
            self.mp_hands.HandLandmark.PINKY_MCP: "little_mcp",
            self.mp_hands.HandLandmark.PINKY_PIP: "little_pip",
            self.mp_hands.HandLandmark.PINKY_DIP: "little_dip",
            self.mp_hands.HandLandmark.PINKY_TIP: "little_tip",
        }

        FINGERNAMES = [
            "wrist",
            "thumb_mcp",
            "thumb_pip",
            "thumb_dip",
            "thumb_tip",
            "index_mcp",
            "index_pip",
            "index_dip",
            "index_tip",
            "middle_mcp",
            "middle_pip",
            "middle_dip",
            "middle_tip",
            "ring_mcp",
            "ring_pip",
            "ring_dip",
            "ring_tip",
            "little_mcp",
            "little_pip",
            "little_dip",
            "little_tip",
        ]
        connections = []
        for i in range(5):
            connections.append((FINGERNAMES[0], FINGERNAMES[i * 4 + 1]))
            connections.append((FINGERNAMES[i * 4 + 1], FINGERNAMES[i * 4 + 2]))
            connections.append((FINGERNAMES[i * 4 + 2], FINGERNAMES[i * 4 + 3]))
            connections.append((FINGERNAMES[i * 4 + 3], FINGERNAMES[i * 4 + 4]))
        self.connections = connections

        self.hand_pose_estimator = self.mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        self.bridge = CvBridge()
        self.pub_img = self.advertise(
            '~output/viz', Image, queue_size=1)
        self.pose_pub = self.advertise('~output/pose',
                                       PeoplePoseArray, queue_size=1)
        self.with_depth = rospy.get_param("~with_depth", True)
        if self.with_depth is True:
            self.pose_3d_pub = self.advertise('~output/pose_3d',
                                           PeoplePoseArray, queue_size=1)
        self.pub_img_compressed = self.advertise(
            '~output/viz/compressed',
            CompressedImage, queue_size=1)
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)

    def subscribe(self):
        if self.with_depth:
            queue_size = rospy.get_param('~queue_size', 10)
            sub_img = message_filters.Subscriber(
                '~input', Image, queue_size=1, buff_size=2**24)
            sub_depth = message_filters.Subscriber(
                '~input/depth', Image, queue_size=1, buff_size=2**24)
            self.subs = [sub_img, sub_depth]

            # NOTE: Camera info is not synchronized by default.
            # See https://github.com/jsk-ros-pkg/jsk_recognition/issues/2165
            sync_cam_info = rospy.get_param("~sync_camera_info", False)
            if sync_cam_info:
                sub_info = message_filters.Subscriber(
                    '~input/info', CameraInfo, queue_size=1, buff_size=2**24)
                self.subs.append(sub_info)
            else:
                self.sub_info = rospy.Subscriber(
                    '~input/info', CameraInfo, self._cb_cam_info)

            if rospy.get_param('~approximate_sync', True):
                slop = rospy.get_param('~slop', 0.1)
                sync = message_filters.ApproximateTimeSynchronizer(
                    fs=self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    fs=self.subs, queue_size=queue_size)
            if sync_cam_info:
                sync.registerCallback(self._cb_with_depth_info)
            else:
                self.camera_info_msg = None
                sync.registerCallback(self._cb_with_depth)
        else:
            sub_img = rospy.Subscriber(
                '~input', Image, self._cb, queue_size=1, buff_size=2**24)
            self.subs = [sub_img]

    def _cb_cam_info(self, msg):
        self.camera_info_msg = msg
        self.sub_info.unregister()
        self.sub_info = None
        rospy.loginfo("Received camera info")

    def unsubscribe(self):
        self.sub.unregister()

    def _cb_with_depth(self, skeleton_msg, depth_msg):
        if self.camera_info_msg is None:
            return
        self._cb_with_depth_info(skeleton_msg, depth_msg, self.camera_info_msg)

    def _cb_with_depth_info(self, img_msg, depth_msg, camera_info_msg):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info_msg)
        bridge = self.bridge
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        depth_img = bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth_img = np.asarray(depth_img, dtype=np.float32)
            depth_img /= 1000  # convert metric: mm -> m
        elif depth_msg.encoding != '32FC1':
            rospy.logerr('Unsupported depth encoding: %s' % depth_msg.encoding)

        H, W = depth_img.shape
        img.flags.writeable = False
        results = self.hand_pose_estimator.process(img)
        people_pose_msg = PeoplePoseArray(header=img_msg.header)
        people_pose_3d_msg = PeoplePoseArray(header=img_msg.header)
        skeleton_msgs = HumanSkeletonArray(header=img_msg.header)
        _PRESENCE_THRESHOLD = 0.5
        _VISIBILITY_THRESHOLD = 0.5

        if results.multi_hand_landmarks:
            fingernames_list = []
            for hand_landmarks in results.multi_hand_landmarks:
                pose_msg = PeoplePose()
                fingernames = []
                image_rows, image_cols, _ = img.shape
                for idx, landmark in enumerate(hand_landmarks.landmark):
                    if ((landmark.HasField('visibility') and
                         landmark.visibility < _VISIBILITY_THRESHOLD) or
                        (landmark.HasField('presence') and
                         landmark.presence < _PRESENCE_THRESHOLD)):
                        continue
                    landmark_px = self.mp_drawing._normalized_to_pixel_coordinates(
                        landmark.x, landmark.y,
                        image_cols, image_rows)
                    if landmark_px:
                        pose_msg.scores.append(landmark.visibility)
                        pose_msg.limb_names.append(self.INDEX2FINGERNAME[idx])
                        pose_msg.poses.append(
                            Pose(position=Point(x=landmark_px[0],
                                                y=landmark_px[1],
                                                z=0)))
                        fingernames.append(self.INDEX2FINGERNAME[idx])
                people_pose_msg.poses.append(pose_msg)
                fingernames_list.append(fingernames)
            hand_list = []
            for hand_pose, fingernames in zip(people_pose_msg.poses, fingernames_list):
                pose_msg = PeoplePose()
                hand = {}
                for scores, limb_names, pose, fingername in zip(
                        hand_pose.scores,
                        hand_pose.limb_names,
                        hand_pose.poses,
                        fingernames):
                    u, v = pose.position.x, pose.position.y
                    if 0 <= u < W and 0 <= v < H:
                        z = float(depth_img[int(v)][int(u)])
                    else:
                        continue
                    if np.isnan(z) or z <= 0:
                        continue
                    x = (u - camera_model.cx()) * z / camera_model.fx()
                    y = (v - camera_model.cy()) * z / camera_model.fy()

                    pose_msg.poses.append(
                        Pose(position=Point(x=x, y=y, z=z),
                             orientation=Quaternion(w=1)))
                    hand[fingername] = np.array([x, y, z])
                people_pose_3d_msg.poses.append(pose_msg)
                hand_list.append(hand)

            for hand in hand_list:
                skeleton_msg = HumanSkeleton(header=img_msg.header)
                for a, b in self.connections:
                    if not (a in hand and b in hand):
                        continue
                    bone_name = '{}->{}'.format(a, b)
                    bone = Segment(
                        start_point=Point(*hand[a]),
                        end_point=Point(*hand[b]))
                    skeleton_msg.bones.append(bone)
                    skeleton_msg.bone_names.append(bone_name)
                skeleton_msgs.skeletons.append(skeleton_msg)
        self.pose_pub.publish(people_pose_msg)
        self.pose_3d_pub.publish(people_pose_3d_msg)
        self.skeleton_pub.publish(skeleton_msgs)

        if self.pub_img.get_num_connections() > 0 or self.pub_img_compressed.get_num_connections() > 0:
            img.flags.writeable = True
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        if self.pub_img.get_num_connections() > 0:
            # Draw the hand annotations on the image.
            out_img_msg = bridge.cv2_to_imgmsg(
                img, encoding='bgr8')
            out_img_msg.header = img_msg.header
            self.pub_img.publish(out_img_msg)

        if self.pub_img_compressed.get_num_connections() > 0:
            # publish compressed http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber  # NOQA
            vis_compressed_msg = CompressedImage()
            vis_compressed_msg.header = img_msg.header
            # image format https://github.com/ros-perception/image_transport_plugins/blob/f0afd122ed9a66ff3362dc7937e6d465e3c3ccf7/compressed_image_transport/src/compressed_publisher.cpp#L116  # NOQA
            vis_compressed_msg.format = 'bgr8' + '; jpeg compressed bgr8'
            vis_compressed_msg.data = np.array(
                cv2.imencode('.jpg', img)[1]).tobytes()
            self.pub_img_compressed.publish(vis_compressed_msg)

    def _cb(self, img_msg):
        bridge = self.bridge

        mp_drawing = self.mp_drawing
        mp_hands = self.mp_hands

        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = self.hand_pose_estimator.process(image)

        people_pose_msg = PeoplePoseArray(header=img_msg.header)
        skeleton_msgs = HumanSkeletonArray(header=img_msg.header)
        _PRESENCE_THRESHOLD = 0.5
        _VISIBILITY_THRESHOLD = 0.5
        if results.multi_hand_landmarks:
            hand_list = []
            for hand_landmarks in results.multi_hand_landmarks:
                pose_msg = PeoplePose()
                image_rows, image_cols, _ = image.shape
                hand = {}
                for idx, landmark in enumerate(hand_landmarks.landmark):
                    if ((landmark.HasField('visibility') and
                         landmark.visibility < _VISIBILITY_THRESHOLD) or
                        (landmark.HasField('presence') and
                         landmark.presence < _PRESENCE_THRESHOLD)):
                        continue
                    landmark_px = self.mp_drawing._normalized_to_pixel_coordinates(
                        landmark.x, landmark.y,
                        image_cols, image_rows)
                    if landmark_px:
                        pose_msg.scores.append(landmark.visibility)
                        pose_msg.limb_names.append(self.INDEX2FINGERNAME[idx])
                        pose_msg.poses.append(
                            Pose(position=Point(x=landmark_px[0],
                                                y=landmark_px[1],
                                                z=0)))
                        hand[self.INDEX2FINGERNAME[idx]] = np.array([landmark_px[0],
                                                                     landmark_px[1],
                                                                     0.0])
                people_pose_msg.poses.append(pose_msg)
                hand_list.append(hand)

            for hand in hand_list:
                skeleton_msg = HumanSkeleton(header=img_msg.header)
                for a, b in self.connections:
                    if not (a in hand and b in hand):
                        continue
                    bone_name = '{}->{}'.format(a, b)
                    bone = Segment(
                        start_point=Point(*hand[a]),
                        end_point=Point(*hand[b]))
                    skeleton_msg.bones.append(bone)
                    skeleton_msg.bone_names.append(bone_name)
                skeleton_msgs.skeletons.append(skeleton_msg)
        self.pose_pub.publish(people_pose_msg)
        self.skeleton_pub.publish(skeleton_msgs)

        if self.pub_img.get_num_connections() > 0 or self.pub_img_compressed.get_num_connections() > 0:
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        if self.pub_img.get_num_connections() > 0:
            # Draw the hand annotations on the image.
            out_img_msg = bridge.cv2_to_imgmsg(
                image, encoding='bgr8')
            out_img_msg.header = img_msg.header
            self.pub_img.publish(out_img_msg)

        if self.pub_img_compressed.get_num_connections() > 0:
            # publish compressed http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber  # NOQA
            vis_compressed_msg = CompressedImage()
            vis_compressed_msg.header = img_msg.header
            # image format https://github.com/ros-perception/image_transport_plugins/blob/f0afd122ed9a66ff3362dc7937e6d465e3c3ccf7/compressed_image_transport/src/compressed_publisher.cpp#L116  # NOQA
            vis_compressed_msg.format = 'bgr8' + '; jpeg compressed bgr8'
            vis_compressed_msg.data = np.array(
                cv2.imencode('.jpg', image)[1]).tobytes()
            self.pub_img_compressed.publish(vis_compressed_msg)


if __name__ == '__main__':
    rospy.init_node('hand_pose_estimation')
    node = HandPoseEstimation()
    rospy.spin()
