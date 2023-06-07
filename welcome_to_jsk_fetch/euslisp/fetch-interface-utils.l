(load "package://fetcheus/fetch-interface.l")

(defmethod fetch-interface
    (:move-to-wait-once
     (&rest args &key (retry 0) (frame-id "world") (wait-for-result-duration-time 0) &allow-other-keys)
                   (let ((ret 0) (count 0) (tm (ros::time-now))
	                           (map-to-frame move-base-goal-map-to-frame)
                             (coords move-base-goal-coords))
                     (when (send self :simulation-modep)
                       ;; wait for-result
                       (while current-goal-coords
                         (send self :robot-interface-simulation-callback))
                       (return-from :move-to-wait t)) ;; simulation-modep
                     (if (null move-base-goal-msg) (return-from :move-to-wait nil))
                     (while (and (not (= ret actionlib_msgs::GoalStatus::*succeeded*))
                                 (not (= ret actionlib_msgs::GoalStatus::*aborted*))
                                 (<= count retry))
                       (send move-base-action :wait-for-result :timeout wait-for-result-duration-time)
                       (ros::ros-info "get-state ~A" (send move-base-action :get-state))
                       (when (eq (send move-base-action :get-state)
                                 actionlib_msgs::GoalStatus::*preempted*)
                         (setq ret actionlib_msgs::GoalStatus::*preempted*)
                         (return))
                       (if (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
                           (setq ret actionlib_msgs::GoalStatus::*succeeded*))

                       (when (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*aborted*) ;; retry
                         (send self :clear-costmap)
                         (send move-base-goal-msg :header :seq count)
                         (send move-base-goal-msg :goal :target_pose :header :seq count)
                         (ros::ros-info "move-to : send-goal to ~A at ~A (~d)" (ros::tf-point->pos (send move-base-goal-msg :goal :target_pose :pose :position)) (send move-base-goal-msg :goal :target_pose :header :frame_id) count)
                         (send move-base-action :send-goal move-base-goal-msg))
                       (incf count 1))
                     ret))
    (:salute-pose ()
                  (send *fetch* :reset-pose)
                  (send *fetch* :head_tilt_joint :joint-angle 83)
                  (send *fetch* :torso_lift_joint :joint-angle 200))
                  
    (:raise-arm-pose ()
                     (send *fetch* :angle-vector #f(377.986 66.8161 49.7995 -124.751 128.303 -49.9474 46.2685 23.8554 -0.141624 31.6808))
                     (send *fetch* :torso_lift_joint :joint-angle 500))
    )
