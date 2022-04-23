;; source ros/pr2_ws/devel/setup.bash                                           
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "handpose")

(pr2-init)
(objects (list *pr2*))

(setq *tfl* (instance ros::transform-listener :init))

(defun give-hand-pose ()
  (send *pr2* :head :angle-vector #f(4.8307 14.694))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
  (setq hand-coords (hand-pose-subscriber))
  (if (> (elt (send hand-coords :pos) 1) 0)
      (setq arm :larm)
      (setq arm :rarm))
  (if (eq arm :larm)
      (progn
        (pr2-tuckarm-pose :larm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 100\
0) (/(+ (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi)))
      (progn
        (pr2-tuckarm-pose :rarm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 100\
0) (/(- (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi))))
  (send *ri* :wait-interpolation)
  (if (eq arm :larm)
      (send *pr2* :larm :inverse-kinematics (make-coords :pos (float-vector -40\
0 600 (elt (send hand-coords :pos) 2))) :rotation-axis nil)
      (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector -40\
0 -600 (elt (send hand-coords :pos) 2))) :rotation-axis nil))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects))

(defun hand-pose-subscriber ()
  (setq hand-pose-array (one-shot-subscribe "/hand_pose_estimation_2d/output/po\
se" jsk_recognition_msgs::HandPoseArray))
  (when (> (length (send hand-pose-array :poses)) 0)
    (setq hand-pose (car (send hand-pose-array :poses)))
    (setq finger-names (send hand-pose :finger_names))
    (when (string= (car finger-names) "wrist")
      ;; (format t "~d ~d ~d~%" (send (car (send hand-pose :poses)) :position :\
x)                                                                              
      ;;         (send (car (send hand-pose :poses)) :position :y)              
      ;;         (send (car (send hand-pose :poses)) :position :z))             
      ;; (return-from hand-pose-subscriber (send (car (send hand-pose :poses)) \
:position :z))                                                                  
      (setq stamp (send hand-pose-array :header :stamp))
      (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_link" (s\
end hand-pose-array :header :frame_id) stamp))
      (when base-to-camera-coords
        ;; (format t "~A~%" base-to-camera-coords)                              
        (setq camera-to-hand-coords (ros::tf-pose->coords (car (send hand-pose \
:poses))))
        (setq base-to-hand (send base-to-camera-coords :transform camera-to-han\
d-coords))
        (return-from hand-pose-subscriber base-to-hand)
      )))
  (hand-pose-subscriber))



