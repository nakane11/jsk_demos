(ros::roseus-add-msgs "hand_navigation_pr2")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "go_action")
(pr2-init nil)

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

(defun go-to-spot (name)
  (ros::ros-info "2.5~%")
  (let* ((ret (get-spot-coords name))
         (goal-pose (car ret))
         (frame-id (cdr ret)))
    (send *ri* :clear-costmap)
    (send *ri* :move-to goal-pose :frame-id frame-id)
    ))

(defclass go-action
  :slots (go-server destination))

(defmethod go-action
  (:init ()
         (ros::rate 10)
         (setq go-server
               (instance ros::simple-action-server
                         :init "go_action"
                         hand_navigation_pr2::GoAction
                         :execute-cb `(lambda-closure nil 0 0 (server goal)
						      (send ,self :callback server goal))))
         self)

  (:callback (server goal)
    (let ((msg (send server :result :result "failed"))
          (request (send goal :goal :request)))
      (cond ((equal request 0)
             (cond ((string-equal (send goal :goal :destination) "center")
                    (ros::ros-info "1")
                    (setq destination "/eng2/7f/room73B2-center")
                    (ros::ros-info "2")
                    (setq res (go-to-spot destination))
                    (when res
                      (setq msg (send server :result :result "succeeded"))))
                   (t nil)))
            ((equal request 1)
             (send *ri* :go-stop)
             (setq msg (send server :result :result "paused")))
            ((equal request 2)
             (if destination
                 (progn
                   (setq res (go-to-spot destination))
                   (if res
                       (setq msg (send server :result :result "succeeded"))
                     (setq msg (send server :result :result "failed"))))
               (setq msg (send server :result :result "no destination"))))
            ((equal request 3)
             (send *ri* :go-stop)
             (setq msg (send server :result :result "cancelled")))
  	  (t nil))
      (send server :set-succeeded msg)))
        
  (:run ()
   (do-until-key
    (ros::spin-once)
    (send go-server :worker)
    (ros::sleep))))

(setq go (instance go-action :init))
(send go :run)

