(ros::roseus-add-msgs "hand_navigation_pr2")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "go_action")
(pr2-init nil)

(defparameter *spots* nil)
(setq *destination* nil)

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

(defun callback (server goal)
  (let (msg)
    (case (send goal :goal :request)
          (0
           (setq msg (send server :result :result "failed"))
           (cond ((string-equal (send goal :goal :destination) "center")
                  (progn
                    (setq *destination* "/eng2/7f/room73B2-center")
                    (ros::ros-info "2")
                    (when (go-to-spot *destination*)
                        (send msg :result "succeeded"))))))
          (1
           (send *ri* :go-stop)
           (setq msg (send server :result :result "paused")))
          (2
           (if *destination*
               (progn
                 (setq res (go-to-spot *destination*))
                 (if res
                     (setq msg (send server :result :result "succeeded"))
                   (setq msg (send server :result :result "failed"))))
             (setq msg (send server :result :result "no destination"))))
          (3
           (send *ri* :go-stop)
           (setq msg (send server :result :result "cancelled"))))
    (send server :set-succeeded msg)))

(defclass go-action
  :slots (go-server))

(defmethod go-action
  (:init ()
         (ros::rate 10)
         (setq go-server
               (instance ros::simple-action-server
                         :init "go_action"
                         hand_navigation_pr2::GoAction
                         :execute-cb #'callback))
         self)
         
  (:run ()
   (do-until-key
    (ros::spin-once)
    (send go-server :worker)
    (ros::sleep))))

(setq go (instance go-action :init))
(send go :run)

