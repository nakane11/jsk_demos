;; source ros/pr2_ws/devel/setup.bash                                           
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")

(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(defun give-hand-pose ()
  (send *pr2* :head :angle-vector #f(0.0 14.694))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
  (setq hand-coords (hand-pose-subscriber))
  (if (> (elt (send hand-coords :pos) 1) 0)
      (setq *arm* :larm)
      (setq *arm* :rarm))
  (if (eq *arm* :larm)
      (progn
        (pr2-tuckarm-pose :larm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 1000) (/(+ (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi)))
      (progn
        (pr2-tuckarm-pose :rarm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 1000) (/(- (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi))))
  (send *ri* :wait-interpolation)
  (if (eq *arm* :larm)
      (progn
        (send *pr2* :larm :inverse-kinematics (make-coords :pos (float-vector -400 600 (elt (send hand-coords :pos) 2))) :rotation-axis nil)
        (send *pr2* :head :angle-vector #f(120.0 26.0)))
      (progn
        (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector -400 -600 (elt (send hand-coords :pos) 2))) :rotation-axis nil)
        (send *pr2* :head :angle-vector #f(-120.0 26.0))))

  ;;(send *pr2* :head :look-at (send *pr2* *arm* :end-coords :worldpos))

  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  (if (eq *arm* :larm)
      (cons 3 (format nil "larm"))
    (cons 3 (format nil "rarm"))))

(defun hand-pose-subscriber ()
  (setq hand-pose-array (one-shot-subscribe "/hand_pose_estimation_2d/output/pose" jsk_recognition_msgs::HandPoseArray))
  (when (> (length (send hand-pose-array :poses)) 0)
    (setq hand-pose (car (send hand-pose-array :poses)))
    (setq finger-names (send hand-pose :finger_names))
    (when (string= (car finger-names) "wrist")                                       
      (setq stamp (send hand-pose-array :header :stamp))
      (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_link" (send hand-pose-array :header :frame_id) stamp))
      (when base-to-camera-coords
        (setq camera-to-hand-coords (ros::tf-pose->coords (car (send hand-pose :poses))))
        (setq base-to-hand (send base-to-camera-coords :transform camera-to-hand-coords))
        (return-from hand-pose-subscriber base-to-hand)
      )))
  (hand-pose-subscriber))



