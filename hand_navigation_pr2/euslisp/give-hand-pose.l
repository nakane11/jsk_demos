;; source ros/pr2_ws/devel/setup.bash                                           
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")

(pr2-init nil)
(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(defun raise-hand (hand-coords)
  (ros::ros-info (format nil "~A" (elt (send hand-coords :pos) 0)))
  (when (< (elt (send hand-coords :pos) 0) 600)
    (if (> (elt (send hand-coords :pos) 1) 0)
        (setq arm "larm")
        (setq arm "rarm"))
    (if (string-equal arm "larm")
        (progn
          (send *pr2* :larm :angle-vector #f(44.3333 36.9321 9.03022 -121.342 -133.409 -92.8499 -19.1599))
          (send *pr2* :head :angle-vector #f(77.0 23.0))
          )
        (progn
          (send *pr2* :rarm :angle-vector #f(-44.3333 36.9321 9.03022 -121.342 133.409 -92.8499 -19.1599))
          (send *pr2* :head :angle-vector #f(-77.0 23.0))
          ))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (ros::set-param "/target_arm" arm)
    (return-from raise-hand t)
    )
  nil)

(defun give-hand-pose (hand-coords)
  (send *pr2* :head :angle-vector #f(0.0 14.694))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (if (> (elt (send hand-coords :pos) 1) 0)
      (setq arm "larm")
      (setq arm "rarm"))
  (if (string-equal arm "larm")
      (progn
        (pr2-tuckarm-pose :larm :inside :single)
        ;; (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 1000) (/(+ (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi)))
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) -135) 1000) (/(+ (elt (send hand-coords :pos) 1) 679) 1000) (rad2deg pi)))
      (progn
        (pr2-tuckarm-pose :rarm :inside :single)
        ;; (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 1000) (/(- (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi))))
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) -135) 1000) (/(- (elt (send hand-coords :pos) 1) 679) 1000) (rad2deg pi))))

  (send *ri* :wait-interpolation)
  (if (string-equal arm "larm")
      (progn
        ;; (send *pr2* :larm :inverse-kinematics (make-coords :pos (float-vector -400 600 (elt (send hand-coords :pos) 2))) :rotation-axis nil)
        ;; (send *pr2* :larm :inverse-kinematics (make-coords :pos (float-vector 135 679 946)) :rotation-axis nil)
        (send *pr2* :larm :angle-vector #f(44.3333 36.9321 9.03022 -121.342 -133.409 -92.8499 -19.1599))
        ;; (send *pr2* :head :angle-vector #f(77.0 23.0))
        )
      (progn
        ;; (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector -400 -600 (elt (send hand-coords :pos) 2))) :rotation-axis nil)
        ;; (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 135 -679 946)) :rotation-axis nil)
        (send *pr2* :rarm :angle-vector #f(-44.3333 36.9321 9.03022 -121.342 133.409 -92.8499 -19.1599))
        ;; (send *pr2* :head :angle-vector #f(-77.0 23.0))
        ))

  ;;(send *pr2* :head :look-at (send *pr2* arm :end-coords :worldpos))

  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;; (send *irtviewer* :draw-objects)
  (ros::set-param "/target_arm" arm)
  t)

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
  nil)



