;; source ros/pr2_ws/devel/setup.bash
(load "package://pr2eus/pr2-interface.l")

(pr2-init)
(objects (list *pr2*))

(defun save-spot ()
  (let ((spot (one-shot-subscribe "/amcl_pose" geometry_msgs::posewithcovariancestamped)))
    (if spot
        (ros::tf-pose->coords (send spot :pose :pose)))))

(defun orient-coords (reference target)
  (let* ((diff (v- (send target :worldpos) (send reference :worldpos)))
         (n-diff (normalize-vector diff)))
    (make-coords :pos (send reference :copy-pos) :rot (orient-x-axis n-diff))))
   
(defun get-rotation-angle (reference target)
  (flet ((get-angle (c) (caar (send c :rpy-angle))))
    (rad2deg (- (get-angle target)
                (get-angle reference)))))

(defun get-diff-position (reference target)
  (/ (elt (send reference :difference-position target) 0) 1000.0))

(defun convert-go-to (reference target)
  (let ((a (orient-coords reference target)))
    (values (get-diff-position a target)
            (get-rotation-angle reference a))))

(defun convert-go-to-robot (target)
  (convert-go-to (save-spot) target))

(defun room-to-corridor-coords()
  (setq s0 (make-coords :pos #f(3710.478 5961.703 0.0) :rpy #f(-3.084 0.0 0.0)))
  (setq s1 (make-coords :pos #f(1980.854 6326.296 0.0) :rpy #f(3.039 0.0 0.0)))
  (setq s2 (make-coords :pos #f(-760.741 6568.424 0.0) :rpy #f(-1.772 0.0 0.0)))
  (setq s3 (make-coords :pos #f(-1341.348 3248.902 0.0) :rpy #f(-1.616 0.0 0.0)))
  (setq s4(make-coords :pos #f(-1303.42 -324.604 0.0) :rpy #f(-1.58 0.0 0.0)))
  (setq s5 (make-coords :pos #f(-1352.215 -4756.532 0.0) :rpy #f(-1.543 0.0 0.0)))
  (setq s6 (make-coords :pos #f(-1154.441 -7620.252 0.0) :rpy #f(-1.709 0.0 0.0)))
  (list s0 s1 s2 s3 s4 s5 s6))

(defun corridor-to-room-coords()
  (setq s0 (make-coords :pos #f(3710.478 5961.703 0.0) :rpy #f(-3.084 0.0 0.0)))
  (setq s1 (make-coords :pos #f(1980.854 6326.296 0.0) :rpy #f(3.039 0.0 0.0)))
  (setq s2 (make-coords :pos #f(-760.741 6568.424 0.0) :rpy #f(-1.772 0.0 0.0)))
  (setq s3 (make-coords :pos #f(-1341.348 3248.902 0.0) :rpy #f(-1.616 0.0 0.0)))
  (setq s4(make-coords :pos #f(-1303.42 -324.604 0.0) :rpy #f(-1.58 0.0 0.0)))
  (setq s5 (make-coords :pos #f(-1352.215 -4756.532 0.0) :rpy #f(-1.543 0.0 0.0)))
  (setq s6 (make-coords :pos #f(-1154.441 -7620.252 0.0) :rpy #f(-1.709 0.0 0.0)))
  (list s6 s5 s4 s3 s2 s1 s0))
  
; pregrasp coords
(defun orient-x-axis (x-axis)
  (let ((mat (make-matrix 3 3)))
    (set-matrix-column mat 0 x-axis)
    (set-matrix-column mat 1 (v* #f(0 0 1) x-axis))
    (set-matrix-column mat 2 #f(0 0 1))
    mat))

(defun go-to-corridor()
  (setq spot-list (room-to-corridor-coords))
  (send *ri* :move-to (pop spot-list) :frame-id "map")
  (dolist (i spot-list)
    (setq tmp (convert-go-to-robot i))
    (send *ri* :go-pos-unsafe 0 0 (cadr tmp))
    (send *ri* :go-pos-unsafe (car tmp) 0 0)))

(defun go-to-room()
  (setq spot-list (corridor-to-room-coords))
  (send *ri* :move-to (pop spot-list) :frame-id "map")
  (dolist (i spot-list)
    (setq tmp (convert-go-to-robot i))
    (send *ri* :go-pos-unsafe 0 0 (cadr tmp))
    (send *ri* :go-pos-unsafe (car tmp) 0 0)))
  
