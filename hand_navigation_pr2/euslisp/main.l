;; source ros/pr2_ws/devel/setup.bash
(ros::roseus-add-srvs "hand_navigation_pr2")
(ros::roseus-add-srvs "topic_tools")
(ros::roseus-add-msgs "geometry_msgs")

(load "package://pr2eus/pr2-interface.l")
(load "package://ros_speech_recognition/euslisp/ros-speech-recognition-client.l")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")

(ros::roseus "hand_navigation")

(pr2-init)
(objects (list *pr2*))
(setq speech-client (instance speech-recognition-client :init))

(defun demo()
  ;; start
  ;; (setq result nil)
  ;; (while (or (null result)
  ;;            (not (string-equal (car result) "テスト")))
  ;;   (format t "waiting speech recognition result ...~%")
  ;;   (setq result (send speech-client :recognize)))

  (wait-for-hotwords speech-client '("一緒に歩こう"))
  (send *ri* :speak-jp "わかりました") 
  ;; set w
  ;; (setq shoulder_width
  ;;       (one-shot-subscribe "/get_people_size/output" std_msgs::Float32
  ;;                           condition: #'(lambda (msg) (> (send msg :data) 0.6))))
  ;; (setq w (* shoulder_width 1.5))
  ;; (send *ri* :speak (format nil "~A meter" shoulder_width))
  (setq shoulder_width 100000.0)
  (while (> shoulder_width 0.6)
    (setq shoulder_width (send (one-shot-subscribe "/get_people_size/output" std_msgs::Float32) :data)))
  (setq w (* shoulder_width 1.5))
  (send *ri* :speak-jp (format nil "~Ameterです" shoulder_width))
  (ros::ros-info "shoulder_width: ~A" shoulder_width)
  
  (wait-for-hotwords speech-client '("手" "つなごう"))
  (send *ri* :speak-jp "わかりました")
  ;; initial pose
;  (setq side :larm)
  (setq side (give-hand-pose)) ;; larm/rarm

  ;; publish boundingbox
  (setq bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
  (send bbox_req :switch t)
  (if (eq side :larm)
      (send bbox_req :position (float-vector -0.2 (+ 0.5 (/ w 2)) 0.0))
      (send bbox_req :position (float-vector -0.2 (- -0.5 (/ w 2)) 0.0)))
  (send bbox_req :dimention (float-vector 0.6 w 3.0))
  (setq bbox_res (ros::service-call "/boundingbox_publisher/set_param" bbox_req))
  (send *ri* :speak-jp "bounding boxをおきました")

  ;; resize footprint
  (if (eq side :larm)
      (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[-0.5,0.5],[-0.5,~A], [0.1,~A], [0.1,0.5], [0.34,0.35],[0.4,0],[0.34,-0.35]]" (+ 0.5 w) (+ 0.5 w)))
      (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35],[0.1,-0.5],[0.1,~A],[-0.5,~A],[-0.5,-0.5]]" (- -0.5 w) (- -0.5 w))))    
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string footprint)
  (send *ri* :speak-jp "footprintを変更しました")

  (setq req (instance topic_tools::MuxSelectRequest :init))
  (send req :topic "/base_scan_filtered_non_human")
  (setq res (ros::service-call "/filter_base_laser_scan/pointcloud_mux/select" req))

  (send req :topic "/tilt_scan_filtered_non_human")
  (setq res (ros::service-call "/filter_tilt_laser_scan/pointcloud_mux/select" req))

  (send *ri* :speak-jp "つかまってください")
  ;; (setq wrench
  ;;       (if (eq side :larm)
  ;;           (one-shot-subscribe "left_endeffector/wrench"
  ;;                           geometry_msgs::WrenchStamped
  ;;                           condition: (lambda (msg) (< (wrenchstamped-to-norm msg) 10)))
  ;;           (one-shot-subscribe "right_endeffector/wrench"
  ;;                           geometry_msgs::WrenchStamped
  ;;                           condition: (lambda (msg) (< (wrenchstamped-to-norm msg) 10)))))

  (if (eq side :larm)
      (setq wrench-topic "/left_endeffector/wrench")
      (setq wrench-topic "/right_endeffector/wrench"))
  (setq norm-prev (wrenchstamped-to-norm (one-shot-subscribe wrench-topic geometry_msgs::WrenchStamped)))
  (setq norm-curr norm-prev)
  (while (< (abs (- norm-curr norm-prev)) 10)
    (setq wrench (one-shot-subscribe wrench-topic geometry_msgs::WrenchStamped))
    (setq norm-curr (wrenchstamped-to-norm wrench))
    (format t "~A~%" norm-curr))
  (send *ri* :speak-jp "どこへいきますか")

  (wait-for-hotwords speech-client '("7階" "廊下" "連れて"))
  (send *ri* :speak-jp "わかりました"))

(defun reset-footprint()
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]"))

(defun wrenchstamped-to-norm(wrenchstamped)
  (setq force (send wrenchstamped :wrench :force))
  (setq norm-curr (expt (+ (expt (send force :x) 2) (expt (send force :y) 2) (expt (send force :z) 2)) 0.5))
  norm-curr)

(defun wait-for-hotwords(client word-list)
  (setq found nil)
  (format t "waiting speech recognition result ...~%")
  (while (not found)
    (setq result (send client :recognize))
    (unless (null result)
      (setq found (search-hotwords (car result) word-list)))))

(defun search-hotwords(target word-list)
  (dolist (i word-list)
    (unless (substringp i target)
      (return-from search-hotwords nil)))
  t)
