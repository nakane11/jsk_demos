(ros::roseus-add-msgs "hand_navigation_pr2")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "give_hand_init_server")
(pr2-init nil)


(defun test()
  (send *ri* :speak-en "test" :wait t))

(defun measuring-shoulder-width(&optional server)
  (let (width 100000.0)
    (while (> width 0.6)
      (if server
          (setq msg (send server :feedback :status "search peoplepose ..."))
          (send server :publish-feedback msg))
      (setq width (send (one-shot-subscribe "/get_people_size/output" std_msgs::Float32) :data)))
    (setq shoulder-width (* width 1.5))
    (send *ri* :speak-en (format nil "~Ameter" shoulder-width))
    (ros::set-param "/shoulder_width" shoulder-width)))

(defun publish-boundingbox(&key shoulder-width target-arm)
  (setq width (or shoulder-width (ros::get-param "/shoulder_width")))
  (setq arm (or target-arm (ros::get-param "/target-arm")))
  
  (setq bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
  (send bbox_req :switch t)
  (if (string-equal arm "larm")
      (send bbox_req :position (float-vector -0.2 (+ 0.5 (/ width 2)) 0.0))
      (send bbox_req :position (float-vector -0.2 (- -0.5 (/ width 2)) 0.0)))
  (send bbox_req :dimention (float-vector 0.6 width 2.0))
  (setq bbox_res (ros::service-call "/boundingbox_publisher/set_param" bbox_req))
  (send *ri* :speak-en "published bounding box" :wait t))

(defun resize-footprint(&key shoulder-width target-arm)
  (setq width (or shoulder-width (ros::get-param "/shoulder_width")))
  (setq arm (or target-arm (ros::get-param "/target-arm")))

  (if (string-equal arm "larm")
      (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[-0.5,0.5],[-0.5,~A], [0.1,~A], [0.1,0.5], [0.34,0.35],[0.4,0],[0.34,-0.35]]" (+ 0.5 width) (+ 0.5 width)))
      (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35],[0.1,-0.5],[0.1,~A],[-0.5,~A],[-0.5,-0.5]]" (- -0.5 width) (- -0.5 width))))    
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string footprint)
  (send *ri* :speak-en "changed footprint" :wait t))

(defun reset-footprint()
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]")
  (send *ri* :speak-en "reset footprint" :wait t))


(defun execute-cb (server goal)
  (let (msg)
    ;; (print (send goal :goal))

    ;; (dotimes (i (send goal :goal :task_id))
    ;;   (setq msg (send server :feedback :status (format nil "~A" i)))
    ;;   (send server :publish-feedback msg)
    ;;   (unix::sleep 1)
    ;;   )
    (ros::ros-info "print")
    (case (send goal :goal :task_id)
          (0 (test))
          (1 (measuring-shoullder-width server))
          (2 (give-hand-pose))
          (3 (publish-boundingbox))
          (4 (resize-footprint))
          (5 (reset-footprint)))
          
    (setq msg (send server :result :success t))
    ;; (send msg :result :type (car result))
    ;; (send msg :result :data (cdr result))
    (send server :set-succeeded msg)
    ))

(setq give-hand-init-server
      (instance ros::simple-action-server
                :init "give_hand_init"
                hand_navigation_pr2::GiveHandInitAction
                :execute-cb #'execute-cb))

(ros::rate 10)
(do-until-key
    ;; (ros::ros-info "running~%") 
 (ros::spin-once)
 (send give-hand-init-server :worker)
 (ros::sleep))


