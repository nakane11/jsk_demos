(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "hand_navigation_pr2")
(ros::roseus-add-srvs "std_srvs")

(ros::roseus "look_for_human")
(ros::wait-for-service "/boundingbox_publisher/set_param")

(setq robot-type (ros::get-param "~robot"))
(when (string= robot-type "pr2")
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init nil)
  ;; (setq *footprint-ns* (list "/move_base_node/global_costmap"
  ;;                            "/move_base_node/local_costmap"
  ;;                            "/safe_teleop_base/local_costmap"))
  (setq *footprint-ns* (ros::get-param "~footprint_ns"))
  (setq *default-footprint* "[0.34,0.35],[0.4,0],[0.34,-0.35],[-0.34,-0.35],[-0.34,0.35]")
  (setq *look-up-angle* #f(85.0 10.0))
  (setq *robot* *pr2*))
(when (string= robot-type "fetch")
  (load "package://fetcheus/fetch-interface.l")
  (fetch-init nil)
  (setq *footprint-ns* (list "/move_base/global_costmap"
                             "/move_base/local_costmap"
                             "/safe_teleop_base/local_costmap"))
  (setq *default-footprint* "[0,0.3],[0.115,0.277],[0.212,0.212],[0.277,0.115],[0.3,0],[0.277,-0.115],[0.212,-0.212],[0.115,-0.277],[0,-0.3],[-0.115,-0.277],[-0.212,-0.212],[-0.277,-0.115],[-0.3,0],[-0.277,0.115],[-0.212,0.212],[-0.115,0.277]")
  (setq *look-up-angle* #f(85.0 -10.0))
  (setq *robot* *fetch*))

(defun get-people ()
  (let (msg (retry-count 0) (max-retry 5))
    (while (< retry-count max-retry)
      (setq msg (one-shot-subscribe "/people_pose_array_accumulator/output" geometry_msgs::PoseArray :timeout 3000))
      (unless msg
        (return-from get-people nil))
      (when msg
        (when (send msg :poses)
          (let ((position (send (elt (send msg :poses) 0) :position)))
            (ros::ros-info "~A" (+ (expt (send position :x) 2) (expt (send position :y) 2)))
            (when (< (+ (expt (send position :x) 2) (expt (send position :y) 2)) 3.6)
              (return-from get-people position)))))
      (ros::ros-info "retry ~A" retry-count)
      (incf retry-count))
    nil))
          
(defun set-bbox ()
  (let ((bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
        position human-footprint footprint (width 0.6))
    (setq position (get-people))
    (if position
        (progn
          (ros::ros-info "true")
          (send bbox_req :switch t)
          (send bbox_req :position (float-vector (send position :x) (send position :y) 0.0))
          (send bbox_req :dimention (float-vector width width 2.0))
          (setq human-footprint (format nil "[~A,~A],[~A,~A],[~A,~A],[~A,~A]"
                                  (- (send position :x) (/ width 2.0)) (- (send position :y) (/ width 2.0))
                                  (- (send position :x) (/ width 2.0)) (+ (send position :y) (/ width 2.0))
                                  (+ (send position :x) (/ width 2.0)) (+ (send position :y) (/ width 2.0))
                                  (+ (send position :x) (/ width 2.0)) (- (send position :y) (/ width 2.0))))
          (setq footprint (format nil "[~A,~A]" *default-footprint* human-footprint)))
        (progn
          (send bbox_req :switch nil)
          (setq footprint (format nil "[~A]" *default-footprint*))))

    (ros::service-call "/boundingbox_publisher/set_param" bbox_req)
    (dolist (ns *footprint-ns*)
      (ros::set-dynamic-reconfigure-param ns "footprint" :string footprint))
    ))

(defun cb (event)
  (ros::service-call "/look_forward_in_navigation/stop" (instance std_srvs::EmptyRequest :init))
  (ros::service-call "/head_look_at/stop" (instance std_srvs::EmptyRequest :init))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :head :angle-vector *look-up-angle*)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (set-bbox)
  (send *robot* :head :angle-vector #f(0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (ros::service-call "/head_look_at/start" (instance std_srvs::EmptyRequest :init))
  (ros::service-call "/look_forward_in_navigation/start" (instance std_srvs::EmptyRequest :init)))

(ros::create-timer 7.0 #'cb)
(ros::spin)
