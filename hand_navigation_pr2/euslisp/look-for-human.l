(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "hand_navigation_pr2")
(ros::roseus "look_for_human")
(pr2-init nil)

(defun set-bbox ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :angle-vector #f(85.0 25.0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (let ((msg (one-shot-subscribe "/people_pose_array_accumulator/output" PoseStamped geometry_msgs :timeout 10))
        (bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init)))
    (if msg
        (progn
          (let ((position (send (send msg :pose) :position)))
            (send bbox_req :switch t)
            (send bbox_req :position (float-vector (send position :x) (send position :y) 0.0))
            (send bbox_req :dimention (float-vector 0.6 0.6 2.0))))
        (send bbox_req :switch nil))
    (ros::service-call "/boundingbox_publisher/set_param" bbox_req)))


(ros::service-call "/look_forward_in_navigation/stop" (instance std_srvs::EmptyRequest :init))
(set-bbox)
(ros::service-call "/look_forward_in_navigation/start" (instance std_srvs::EmptyRequest :init))
