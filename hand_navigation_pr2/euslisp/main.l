;; source ros/pr2_ws/devel/setup.bash
(ros::roseus-add-srvs "hand_navigation_pr2")
(load "package://pr2eus/pr2-interface.l")
(load "package://hand_navigation_pr2/give-hand-pose.l")

(pr2-init)
(objects (list *pr2*))

(ros::roseus "hand_nav")
(ros::wait-for-service "get_people_size")
(setq req (instance hand_navigation_pr2::PeopleSize :init))
;(setq res (ros::service-call "get_people_size" req))

(setq side (give-hand-pose)) ;; :larm or :rarm

(if (eq side :larm)
    (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[-0.4, 0.4], [-0.4, 0.6], [0, 0.6], [0, 0.4], [0.34,0.35],[0.4,0],[0.34,-0.35]]")
    (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35],[0,-0.4],[0,-0.6],[-0.4,-0.6],[-0.4,-0.4]]"))



(defun reset-footprint()
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]"))
  
