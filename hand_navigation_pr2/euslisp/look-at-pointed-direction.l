(ros::roseus-add-msgs "geometry_msgs")
(load "package://pr2eus/pr2-interface.l")

(defun cb (msg)
  (setq stamp (send msg :header :stamp))
  (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_link" (send msg :header :frame_id) stamp))
  (when base-to-camera-coords
    (let ((poses (send msg :poses)))
      (when (> (length poses) 0)
        (setq camera-to-centroid (ros::tf-pose->coords (car poses)))
        (setq point (send base-to-camera-coords :transform camera-to-centroid))
        (send *pr2* :head :look-at (send point :pos))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)))))

(ros::roseus "look_at")
(pr2-init nil)
(ros::subscribe "/cluster_point_indices_decomposer/centroid_pose_array" geometry_msgs::PoseArray #'cb)
(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))
(ros::rate 1)
(do-until-key
    (ros::spin-once)
    (ros::sleep))

  
