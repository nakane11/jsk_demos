(ros::load-ros-manifest "geometry_msgs")

(defclass virtual-force-client
    :slots (sub wrench-msg))


(defmethod virtual-force-client
  (:init (topic-name)
         (ros::ros-info "init")
   (setq sub (ros::subscribe
              topic-name geometry_msgs::WrenchStamped
              #'send self :callback)))
  (:callback (msg)
             (setq wrench-msg msg))

  (:result-torque ()
           (unless (null wrench-msg)
             (send (send wrench-msg :wrench) :torque)))

  (:result-force ()
           (unless (null wrench-msg)
             (send (send wrench-msg :wrench) :force))))
