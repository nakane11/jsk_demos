;; source ros/pr2_ws/devel/setup.bash
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")

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

(defun give-hand-pose (&optional (arm :rarm))
  (setq hand-coords (hand-pose-subscriber))
  (if (> (elt (send hand-coords :pos) 1) 0)
      (progn
        (pr2-tuckarm-pose :larm :inside :single)
        (send *pr2* :translate (float-vector (- (elt (send hand-coords :pos) 0) 400) (+ (elt (send hand-coords :pos) 1) 600) 0)))
      (progn
        (pr2-tuckarm-pose :rarm :inside :single)
        (send *pr2* :translate (float-vector (- (elt (send hand-coords :pos) 0) 400) (- (elt (send hand-coords :pos) 1) 600) 0))))
  (send *pr2* :rotate pi :z)
  (send *ri* :move-to (send *pr2* :copy-worldcoords))
  (send *ri* :wait-interpolation)
  (if (> (elt (send hand-coords :pos) 1) 0)
      (send *pr2* :larm :inverse-kinematics hand-coords :rotation-axis nil)
      (send *pr2* :rarm :inverse-kinematics hand-coords :rotation-axis nil))    
  (send hand-coords :draw-on :size 300 :flush t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects))
  

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


;; (send *pr2* :reset-pose)
;; (send *pr2* :translate #f(500 0 0))
;; (send *pr2* :larm :inverse-kinematics hand-coords :rotation-axis nil)

(defun visualize-ik (model arm  &key (x-range `(-1000 1000)) (y-range `(-600 1400)) (z-range `(0 1600)) (interval 400))
  (setq point-list (list *pr2*))
  (dotimes (i (+ 1 (floor (/ (- (cadr x-range) (car x-range)) interval))))
    (setq x (+ (car x-range) (* interval i)))
    (dotimes (j (+ 1 (floor (/ (- (cadr y-range) (car y-range)) interval))))
      (setq y (+ (car y-range) (* interval j)))
      (dotimes (k (+ 1 (floor (/ (- (cadr z-range) (car z-range)) interval))))
        (setq z (+ (car z-range) (* interval k)))
        (setq ik-target (make-coords :pos (float-vector x y z)))
          (if (send model arm :inverse-kinematics ik-target :rotation-axis nil)
              (push (make-point x y z :green) point-list)
              (push (make-point x y z :red) point-list)))))
  (objects point-list)
  (send *irtviewer* :draw-objects))

  
(defun make-point (x y z color)
  (setq point (make-gdome (make-gdome (make-gdome (make-icosahedron 10.0)))))
  (send point :set-color color)
  (send point :locate (float-vector x y z))
  point)


  
