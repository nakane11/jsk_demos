(ros::roseus-add-msgs "hand_navigation_pr2")
(ros::roseus-add-msgs "move_base_msgs")
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
  (let* ((ret (get-spot-coords name))
         (goal-pose (car ret))
         (frame-id (cdr ret)))
    (send *ri* :clear-costmap)
    (send *ri* :move-to goal-pose :no-wait t :frame-id frame-id)
    ))

(defclass go-action
  :slots (destination))

(defmethod go-action
  (:init ()
   (ros::rate 10)
   (ros::advertise "/go/result" std_msgs::string 1)
   (ros::subscribe "/go/request" hand_navigation_pr2::GoRequest #'send self :req-cb)
   (ros::subscribe "/move_base/result" move_base_msgs::MoveBaseActionResult #'send self :move-base-cb)
   self)

  (:move-base-cb (msg)
                 (setq result (instance std_msgs::string :init))
                 (setq s (send msg :status :status))
                 (cond
                   ((eq s 2)
                    (send result :data "move-base cancel executed"))
                   ((eq s 3)
                    (send result :data "move-base succeeded"))
                   ((eq s 4) (send result :data "move-base failed"))
                   (t (return-from :move-base-cb nil)))
                 (ros::publish "/go/result" result))
                 
  (:req-cb (msg)
    (let ((result (instance std_msgs::string :init))
          (request (send msg :request)))
      (case request
        (0
         (send *ri* :go-stop)
         (setq destination nil)
         (send result :data "move-base reset called")))
        (1
         (block b
           (cond ((string-equal (send msg :destination) "center")
                  (setq destination "/eng2/7f/room73B2-center")
                  (setq res (go-to-spot destination)))
                 (t
                  (send result :data "cannot find destination")
                  (return-from b nil)))
           (send result :data "move-base started")))
        (2
         (send *ri* :go-stop)
         (send result :data "move-base paused"))
        (3
         (if destination
             (progn
               (setq res (go-to-spot destination))
               (send result :data "move-base started"))
             (send result :data "no destination")))

        (ros::publish "/go/result" result)))
        
  (:run ()
   (do-until-key
    (ros::spin-once)
    (ros::sleep))))

(setq go (instance go-action :init))
(send go :run)

