;; source ros/pr2_ws/devel/setup.bash
(ros::roseus-add-srvs "hand_navigation_pr2")
(ros::roseus-add-srvs "topic_tools")
(load "package://pr2eus/pr2-interface.l")
(load "package://hand_navigation_pr2/give-hand-pose.l")

(pr2-init)
(objects (list *pr2*))

(ros::roseus "hand_navigation")
(ros::wait-for-service "get_people_size")
(setq ps_req (instance hand_navigation_pr2::PeopleSizeRequest :init))
(setq ps_res (ros::service-call "get_people_size" ps_req))
(setq w (* (send ps_res :width) 1.5))

(setq side (give-hand-pose)) ;; :larm or :rarm

(setq bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
(send bbox_req :switch t)
(if (eq side :larm)
    (send bbox_req :position (float-vector -0.2 (+ 0.5 (/ w 2)) 0.0))
    (send bbox_req :position (float-vector -0.2 (- -0.5 (/ w 2)) 0.0)))
(send bbox_req :dimention (float-vector 0.6 w 3.0))
(setq bbox_res (ros::service-call "boubdingbox_publisher_set_param" bbox_req))

(if (eq side :larm)
    (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[-0.5,0.5],[-0.5,~A], [0.1,~A], [0.1,0.5], [0.34,0.35],[0.4,0],[0.34,-0.35]]" (+ 0.5 w) (+ 0.5 w)))
    (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35],[0.1,-0.5],[0.1,~A],[-0.5,~A],[-0.5,-0.5]]" (- -0.5 w) (- -0.5 w))))    
(ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string footprint)

(setq req (instance topic_tools::MuxSelectRequest :init))
(send req :topic "/base_scan_filtered_non_human")
(setq res (ros::service-call "/filter_base_laser_scan/pointcloud_mux/select" req))

(defun reset-footprint()
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]"))
  
