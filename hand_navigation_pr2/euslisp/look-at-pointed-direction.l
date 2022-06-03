(ros::roseus-add-msgs "geometry_msgs")
(load "package://pr2eus/pr2-interface.l")

(defun cb (msg)
  (ros::ros-info "cb")
  (setq stamp (send msg :header :stamp))
  (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_footprint" (send msg :header :frame_id) stamp))
  (when base-to-camera-coords
    (let ((poses (send msg :poses)) (r 1.5))
      (when (> (length poses) 0)
        (setq camera-to-centroid (ros::tf-pose->coords (car poses)))
        (setq point (send base-to-camera-coords :transform camera-to-centroid))
        
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
        (setq current (send *pr2* :head :angle-vector))
        (send *pr2* :head :look-at (send point :pos))
        (setq target (send *pr2* :head :angle-vector))
        
        (setq move (float-vector (+ (elt current 0) (/ (- (elt target 0) (elt current 0)) r))
                                 (+ (elt current 1) (/ (- (elt target 1) (elt current 1)) r))))
        (send *pr2* :head :angle-vector move)
        (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)))))

(ros::roseus "look_at")
(pr2-init nil)
(ros::subscribe "/cluster_point_indices_decomposer/centroid_pose_array" geometry_msgs::PoseArray #'cb)
(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))
(ros::rate 10)
(do-until-key
    (ros::spin-once)
    (ros::sleep))

  
