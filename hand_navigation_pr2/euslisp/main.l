;; source ros/pr2_ws/devel/setup.bash
(ros::roseus-add-srvs "hand_navigation_pr2")
(ros::roseus-add-srvs "topic_tools")
(load "package://pr2eus/pr2-interface.l")
(load "package://ros_speech_recognition/euslisp/ros-speech-recognition-client.l")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")

(ros::roseus "hand_navigation")

(pr2-init)
(objects (list *pr2*))
(setq speech-client (instance speech-recognition-client :init))

;; start
;; (setq result nil)
;; (while (or (null result)
;;            (not (string-equal (car result) "テスト")))
;;   (format t "waiting speech recognition result ...~%")
;;   (setq result (send speech-client :recognize)))

(wait-for-hotwords speech-client '("手" "つなごう"))

;; set w
(setq shoulder_width
      (one-shot-subscribe "/get_people_size/output" std_msgs::Float32
                          condition: (lambda (msg) (> (send msg :data) 0.6))))
(setq w (* shoulder_width 1.5))
(send *ri* :speak (format nil "~A meter" shoulder_width))
;; (while (> shoulder_width 0.6)
;;   (setq shoulder_width (send (one-shot-subscribe "/get_people_size/output" std_msgs::Float32) :data)))
;; (setq w (* shoulder_width 1.5))
  
;; initial pose
(setq side (give-hand-pose)) ;; larm/rarm

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

(send *ri* :speak "hold on")
(setq wrench
      (if (eq side :larm)
          (one-shot-subscribe "left_endeffector/wrench"
                          geometry_msgs::WrenchStamped
                          condition: (lambda (msg) (< (wrenchstamped-to-norm msg) 10)))
          (one-shot-subscribe "right_endeffector/wrench"
                          geometry_msgs::WrenchStamped
                          condition: (lambda (msg) (< (wrenchstamped-to-norm msg) 10)))))
(send *ri* :speak "hand grasped")
;; (setq norm 0)
;; (while (< norm 10)
;;   (setq wrench (one-shot-subscribe "left_endeffector/wrench" geometry_msgs::WrenchStamped))
;;   (setq norm (wrenchstamped-to-norm wrench)))

;; (setq result nil)
;; (while (or (null result)
;;            (not (string-equal (car result) "テスト")))
;;   (format t "waiting speech recognition result ...~%")
;;   (setq result (send speech-client :recognize)))

(wait-for-hotwords speech-client '("7階" "廊下" "連れて"))

(defun reset-footprint()
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]"))

(defun wrenchstamped-to-norm(wrenchstamped)
  (setq force (send wrenchstamped :wrench :force))
  (setq norm (expt (+ (expt (send force :x) 2) (expt (send force :y) 2) (expt (send force :z) 2)) 0.5))
  norm)

(defun wait-for-hotwords(client word-list)
  (let (found nil)
    (while (not found)
      (setq result (send client :recognize))
      (unless (null result)
        (setq found (search-hotwords (car result) word-list))))))
           
(defun search-hotwords(target word-list)
  (dolist (i word-list)
    (unless (substringp i target)
      (return-from search-hotwords nil)))
  t)
