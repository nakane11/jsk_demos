;; source ros/pr2_ws/devel/setup.bash
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_demo_common/euslisp/pr2-action.l")

(ros::roseus "handpose")

(pr2-init)
(objects (list *pr2*))

(setq *tfl* (instance ros::transform-listener :init))

(defparameter *spots* nil)
(defun get-spot-coords (name)
  (unless *spots*
    (setq *spots* (one-shot-subscribe "/spots_marker_array" visualization_msgs::MarkerArray)))
  (let ((spot-coords nil) (frame-id nil))
    (dolist (x (send *spots* :markers))
      (if (equal (send x :text) name)
        (progn
          (setq spot-coords (send x :pose))
          (setq frame-id (send (send x :header) :frame_id)))))
    (send (send spot-coords :position) :z 0)
    (setq spot-coords (ros::tf-pose->coords spot-coords))
    (cons spot-coords frame-id)))


(defun goto-spot (name)
  (let* ((ret (get-spot-coords name))(goal-pose (car ret)) (frame-id (cdr ret)))
    (send *ri* :clear-costmap)
    (send *ri* :move-to goal-pose :frame-id frame-id)
    ))

(defun give-hand-pose ()
  ;; (load "models/arrow-object.l")
  ;; (setq origin (arrow))
  ;; (setq hand-arrow (arrow))             ;
  (send *pr2* :head :angle-vector #f(4.8307 14.694))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
  (setq hand-coords (hand-pose-subscriber))
  (if (> (elt (send hand-coords :pos) 1) 0)
      (setq arm :larm)
      (setq arm :rarm))
  ;; (send hand-arrow :newcoords hand-coords)
  ;; (objects (list *pr2* origin hand-arrow))
  (if (eq arm :larm)
      (progn
        (pr2-tuckarm-pose :larm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 1000) (/(+ (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi)))
      (progn
        (pr2-tuckarm-pose :rarm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) 400) 1000) (/(- (elt (send hand-coords :pos) 1) 600) 1000) (rad2deg pi))))
  (send *ri* :wait-interpolation)
  (if (eq arm :larm)
      (send *pr2* :larm :inverse-kinematics (make-coords :pos (float-vector -400 600 (elt (send hand-coords :pos) 2))) :rotation-axis nil)
      (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector -400 -600 (elt (send hand-coords :pos) 2))) :rotation-axis nil))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  ;; (while t
  ;;   (when (wait-for-hand-impact arm)
  ;;       (return)))
  ;; (send *ri* :speak "Hello")
  ;(goto-spot "/eng2/7f/room73B2-center")
  )
  

(defun hand-pose-subscriber ()
  (setq hand-pose-array (one-shot-subscribe "/hand_pose_estimation_2d/output/pose" jsk_recognition_msgs::HandPoseArray))
  (when (> (length (send hand-pose-array :poses)) 0)
    (setq hand-pose (car (send hand-pose-array :poses)))
    (setq finger-names (send hand-pose :finger_names))
    (when (string= (car finger-names) "wrist")
      ;; (format t "~d ~d ~d~%" (send (car (send hand-pose :poses)) :position :x)
      ;;         (send (car (send hand-pose :poses)) :position :y)
      ;;         (send (car (send hand-pose :poses)) :position :z))
      ;; (return-from hand-pose-subscriber (send (car (send hand-pose :poses)) :position :z))
      (setq stamp (send hand-pose-array :header :stamp))
      (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_link" (send hand-pose-array :header :frame_id) stamp))
      (when base-to-camera-coords
        ;; (format t "~A~%" base-to-camera-coords)
        (setq camera-to-hand-coords (ros::tf-pose->coords (car (send hand-pose :poses))))
        (setq base-to-hand (send base-to-camera-coords :transform camera-to-hand-coords))
        (return-from hand-pose-subscriber base-to-hand)
      )))
  (hand-pose-subscriber))



