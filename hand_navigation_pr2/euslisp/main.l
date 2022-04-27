;; source ros/pr2_ws/devel/setup.bash
(ros::roseus-add-srvs "hand_navigation_pr2")
(ros::roseus-add-srvs "topic_tools")
(load "package://pr2eus/pr2-interface.l")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://ros_speech_recognition/euslisp/ros-recognition-client.l")

(ros::roseus "hand_navigation")

(pr2-init)
(objects (list *pr2*))
(setq speech-client (instance speech-recognition-client :init))
(setq result (send speech-client :recognize))
;; set w
(setq shoulder_width 100000.0)
(while (> shoulder_width 0.6)
  (setq shoulder_width (send (one-shot-subscribe "/get_people_size/output" std_msgs::Float32) :data)))
(setq w (* shoulder_width 1.5))
(send *ri* :speak (format nil "~A meter" shoulder_width))
;; initial pose
(setq side (give-hand-pose)) ;; :larm or :rarm

;; publish boundingbox
(setq bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
(send bbox_req :switch t)
(if (eq side :larm)
    (send bbox_req :position (float-vector -0.2 (+ 0.5 (/ w 2)) 0.0))
    (send bbox_req :position (float-vector -0.2 (- -0.5 (/ w 2)) 0.0)))
(send bbox_req :dimention (float-vector 0.6 w 3.0))
(setq bbox_res (ros::service-call "/boundingbox_publisher/set_param" bbox_req))

;; resize footprint
(if (eq side :larm)
    (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[-0.5,0.5],[-0.5,~A], [0.1,~A], [0.1,0.5], [0.34,0.35],[0.4,0],[0.34,-0.35]]" (+ 0.5 w) (+ 0.5 w)))
    (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35],[0.1,-0.5],[0.1,~A],[-0.5,~A],[-0.5,-0.5]]" (- -0.5 w) (- -0.5 w))))    
(ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string footprint)

;; (setq req (instance topic_tools::MuxSelectRequest :init))
;; (send req :topic "/base_scan_filtered_non_human")
;; (setq res (ros::service-call "/filter_base_laser_scan/pointcloud_mux/select" req))

;; (send req :topic "/tilt_scan_filtered_non_human")
;; (setq res (ros::service-call "/filter_tilt_laser_scan/pointcloud_mux/select" req))

(send *ri* :speak (format nil "hold on"))
(setq norm 0)
(while (< norm 5)
  (setq force (send (one-shot-subscribe "left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench :force))
  (setq norm (expt (+ (expt (send force :x) 2) (expt (send force :y) 2) (expt (send force :z) 2)) 0.5)))


(defun reset-footprint()
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]"))
  
