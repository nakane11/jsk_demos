;; source ros/pr2_ws/devel/setup.bash
(load "package://pr2eus/pr2-interface.l")

(pr2-init)
(objects (list *pr2*))

;#<coordinates #X55f3d564ecf0  3710.478 5961.703 0.0 / -3.084 0.0 0.0>
(defun demo()
  (send *ri* :move-to (make-coords :pos #f(3710.478 5961.703 0.0) :rpy #f(-3.084 0.0 0.0)))
  (send *ri* :go-pos-unsafe 0 0 -7.903) ;#<coordinates #X55f3d3567b10  1574.62 6133.688 0.0 / 2.949 0.0 0.0>
  (send *ri* :go-pos-unsafe 2.14277 0 0)
  (send *ri* :go-pos-unsafe 0 0 7.86723);#<coordinates #X55f3d35657b8  -760.741 6568.424 0.0 / -1.772 0.0 0.0>
  (send *ri* :go-pos-unsafe 3.86617 0 0)
  (send *ri* :go-pos-unsafe ) ;#<coordinates #X55f3d3561d08  -1341.348 3248.902 0.0 / -1.616 0.0 0.0>
  (send *ri* :go-pos-unsafe ) ;#<coordinates #X55f3d3572500  -1303.42 -324.604 0.0 / -1.58 0.0 0.0>
  (send *ri* :go-pos-unsafe ) ;#<coordinates #X55f3d356e720  -1352.215 -4756.532 0.0 / -1.543 0.0 0.0>
  (send *ri* :go-pos-unsafe )) ;#<coordinates #X55f3d356abc8  -1154.441 -7620.252 0.0 / -1.709 0.0 0.0>

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

(defun define-coords()
  (setq s0 (make-coords :pos #f(3710.478 5961.703 0.0) :rpy #f(-3.084 0.0 0.0)))
  (setq s1 (make-coords :pos #f(1574.62 6133.688 0.0) :rpy #f(2.949 0.0 0.0)))
  (setq s2 (make-coords :pos #f(-760.741 6568.424 0.0) :rpy #f(-1.772 0.0 0.0)))
  (setq s3 (make-coords :pos #f(-1341.348 3248.902 0.0) :rpy #f(-1.616 0.0 0.0)))
  (setq s4(make-coords :pos #f(-1303.42 -324.604 0.0) :rpy #f(-1.58 0.0 0.0)))
  (setq s5 (make-coords :pos #f(-1352.215 -4756.532 0.0) :rpy #f(-1.543 0.0 0.0)))
  (setq s6 (make-coords :pos #f(-1154.441 -7620.252 0.0) :rpy #f(-1.709 0.0 0.0))))
  
; pregrasp coords
(defun orient-x-axis (x-axis)
  (let ((mat (make-matrix 3 3)))
    (set-matrix-column mat 0 x-axis)
    (set-matrix-column mat 1 (v* #f(0 0 1) x-axis))
    (set-matrix-column mat 2 #f(0 0 1))
    mat))
