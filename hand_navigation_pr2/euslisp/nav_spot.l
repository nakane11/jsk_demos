;; source ros/pr2_ws/devel/setup.bash
(load "package://pr2eus/pr2-interface.l")

(pr2-init)
(objects (list *pr2*))
;;(pr2-tuckarm-pose)

(defun test-1 ()
  (dotimes (i 10)
    (setq theta 0.15708) ;;[m/sec] [m/sec] [rad/sec] &optional (msec 1000) ;; msec is total animation time [msec] &key (stop t) (wait))

    (send *ri* :go-velocity 0 0 theta 1000 :wait t)
    (print (list 'send 0 0 theta))
    ))

(defun test-2 ()
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000) ;; no-blocking
  (send *ri* :wait-interpolation) ;; block
  (send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

;;
;; (load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
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


;;; (pr2-tuckarm-pose)
;; (let* ((ret (get-spot-coords "/eng2/7f/room73B2-center"))(goal-pose (car ret)) (frame-id (cdr ret))) (send *ri* :move-to goal-pose :frame-id frame-id))

;; (send *ri* :clear-costmap)
;; (send *ri* :change-inflation-range 0.1)
