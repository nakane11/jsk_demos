;; source ros/pr2_ws/devel/setup.bash
(load "package://pr2eus/pr2-interface.l")

(pr2-init)
(objects (list *pr2*))

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


(defun one-tuckarm-pose (&optional (tuckarm :rarm))
  (let* ((current-arm (check-tuckarm-pose :thre 40)) ;; nil rarm larm
         (msec 2000))
    
    (format t "current-arm: ~A~%" current-arm)
    
      (when (or (eq current-arm tuckarm) (not current-arm))
	(progn
	  (send *pr2* :larm :angle-vector #f( 25 0 0 -121 0 -6 0))
	  (send *pr2* :rarm :angle-vector #f(-25 0 0 -121 0 -6 0))
	  (send *ri* :angle-vector (send *pr2* :angle-vector) msec)
	  (send *ri* :wait-interpolation)
	  ))

    (if (eq tuckarm :rarm)
        (progn
          (send *pr2* :rarm :angle-vector (get-tuckarm :larm :inside :rarm))
          (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) msec))
      (progn
        (send *pr2* :larm :angle-vector (get-tuckarm :rarm :inside :larm))
        (send *ri* :larm-angle-vector (send *pr2* :angle-vector) msec)))
    (send *ri* :wait-interpolation)

    t
    ))


(defun init-pose (&key (arm :rarm) &key (h 0.0))
  (one-tuckarm-pose arm)

  (if (eq arm :rarm)
      (progn
	(setq ik-target (make-coords :pos (float-vector -650 430.5 (+ 800 h))))
	(send *pr2* :larm :inverse-kinematics ik-target :rotation-axis nil))
      (progn
	(setq ik-target (make-coords :pos (float-vector -650 -430.5 (+ 800 h))))
	(send *pr2* :rarm :inverse-kinematics ik-target :rotation-axis nil)))
      
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects))
  

