(ros::roseus-add-msgs "geometry_msgs")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "wrench_reader")

(pr2-init)
(defun timer-cb(event)
  (when (> (abs *norm*) 30)
    (send *ri* :speak (format nil "~A" *norm*))))


(defun subscribe-cb (msg)
  (setq force (send msg :wrench :force))
  (setq *norm* (expt (+ (expt (send force :x) 2) (expt (send force :y) 2) (expt (send force :z) 2)) 0.5)))

(ros::create-timer 5.0 #'timer-cb)
(ros::subscribe "left_endeffector/wrench" geometry_msgs::WrenchStamped #'subscribe-cb)
(ros::spin)
  
