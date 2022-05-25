(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")

(defun callback (msg)
  (ros::ros-info "cb")
  (unless (ros::has-param "/target_arm")
    (return-from callback))
  (let ((arm (ros::get-param "/target_arm")) hand-pose finger-pose  stamp base-to-camera-coords camera-to-hand-coords human-hand-coords robot-hand-coords diff (grasped (instance std_msgs::bool :init)))
    (cond
      ((string= arm "larm") (setq arm :larm))
      ((string= arm "rarm") (setq arm :rarm))
      (t (return-from callback)))
    (when (> (length (send msg :poses)) 0)
      (setq hand-pose (car (send msg :poses)))
      (when (> (length (send hand-pose :poses)) 0)
        (setq finger-pose (car (send hand-pose :poses)))
        (setq stamp (send msg :header :stamp))
        (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_footprint" (send msg :header :frame_id) stamp))
        (when base-to-camera-coords
          (setq camera-to-hand-coords (ros::tf-pose->coords finger-pose))
          (setq human-hand-coords (send base-to-camera-coords :transform camera-to-hand-coords))
          (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
          (setq robot-hand-coords (send *pr2* arm :end-coords))
          (setq diff (norm (send (send human-hand-coords :transformation robot-hand-coords) :worldpos)))
          
          (ros::ros-info "~A" diff)
          (send *ri* :speak-jp (format nil "~A" diff) :wait t)
          (when (< diff 200)
            (send grasped :data t)))))
    (ros::publish "/grasped" grasped)))
    

(ros::roseus "grasp_detector")
(pr2-init nil)
(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))
(ros::advertise "/grasped" std_msgs::bool 1)
(ros::subscribe "/hand_pose_estimation_2d/output/pose" jsk_recognition_msgs::HandPoseArray #'callback)

(ros::rate 0.3)
(do-until-key
    (ros::spin-once)
  (ros::sleep))


