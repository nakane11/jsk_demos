(ros::roseus-add-msgs "hand_navigation_pr2")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "give_hand_init_server")

(defun executwe-cb (server goal)
  (let (msg result)
    ;; (print (send goal :goal))

    ;; (dotimes (i (send goal :goal :task_id))
    ;;   (setq msg (send server :feedback :status (format nil "~A" i)))
    ;;   (send server :publish-feedback msg)
    ;;   (unix::sleep 1)
    ;;   )

    (case (send goal :goal :task_id)
          (1 (setq result (measuring-shoullder-width)))
          (2 (setq result (give-hand-pose))))
          
    (setq msg (send server :result :success t))
    (send msg :result :type (car result))
    (send msg :result :data (cdr result))
    (send server :set-succeeded msg)
    ))

(setq hand-navigation-motion-server
      (instance ros::simple-action-server
                :init "give_hand_init"
                hand_navigation_pr2::HandNavigationAction
                :execute-cb #'execute-cb))

(ros::rate 10)
(do-until-key
    ;; (ros::ros-info "running~%") 
 (ros::spin-once)
 (send hand-navigation-motion-server :worker)
 (ros::sleep))

(defun measuring-shoulder-width()
  (let (shoulder_width 100000.0)
    (while (> shoulder_width 0.6)
      (setq shoulder_width (send (one-shot-subscribe "/get_people_size/output" std_msgs::Float32) :data)))
    (setq *width* (* shoulder_width 1.5))
    (send *ri* :speak-jp (format nil "~Ameterです" shoulder_width))
    (ros::ros-info "shoulder_width: ~A" shoulder_width))
  (cons 2 (format nil "~A" *width*)))

(defun publish-boundingbox(&rest arg)
  (setq target-width (or (car arg) *width*))
  (unless (target-width)
    (return-from publish-boundingbox (cons 99 "width not defined")))
  
  (setq bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
  (send bbox_req :switch t)
  (if (eq side :larm)
      (send bbox_req :position (float-vector -0.2 (+ 0.5 (/ w 2)) 0.0))
    (send bbox_req :position (float-vector -0.2 (- -0.5 (/ w 2)) 0.0)))
  (send bbox_req :dimention (float-vector 0.6 w 2.0))
  (setq bbox_res (ros::service-call "/boundingbox_publisher/set_param" bbox_req))
  (send *ri* :speak-en "published bounding box" :wait t)
  (cons 0 "t"))

(defun resize-footprint(&key input-arm input-width)
  (setq target-arm (or (input-arm) *arm*))
  (unless (target-arm)
    (return-from publish-boundingbox (cons 99 "arm not defined")))
  (setq target-width (or (input-width) *width*))
  (unless (target-width)
    (return-from publish-boundingbox (cons 99 "width not defined")))

  (if (eq target-arm :larm)
      (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[-0.5,0.5],[-0.5,~A], [0.1,~A], [0.1,0.5], [0.34,0.35],[0.4,0],[0.34,-0.35]]" (+ 0.5 target-width) (+ 0.5 target-width)))
      (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35],[0.1,-0.5],[0.1,~A],[-0.5,~A],[-0.5,-0.5]]" (- -0.5 target-width) (- -0.5 target-width))))    
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string footprint)
  (send *ri* :speak-en "changed footprint" :wait t)
  (cons 0 "t"))

(defun reset-footprint()
  (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]")
  (cons 0 "t"))

