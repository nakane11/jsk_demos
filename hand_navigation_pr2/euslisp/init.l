(ros::roseus-add-msgs "hand_navigation_pr2")
(ros::roseus-add-srvs "hand_navigation_pr2")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "give_hand_init")
(pr2-init nil)


(defclass init-server
    :slots (shoulder-width hand-coords pub-bbox fp is-cancelled))

(defmethod init-server
    (:reset ()
     (setq shoulder-width nil)
     (when (ros::has-param "/target_arm")
       (ros::delete-param "/target_arm"))
     (setq bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
     (send bbox_req :switch nil)
     (setq bbox_res (ros::service-call "/boundingbox_publisher/set_param" bbox_req))
     (setq pub-bbox nil)
     (setq fp nil)
     (send self :reset-footprint)
     t)
    
  (:set-shoulder-width ()
                       (when shoulder-width
                         (return-from :set-shoulder-width t))
                       (send *ri* :speak-jp "正面に立ってください" :wait t)
                       (let (width 100000.0)
                         (while (> width 0.6)
                           (if is-cancelled
                               (return-from :set-shoulder-width nil)
                               (setq width (send (one-shot-subscribe "/get_people_size/output" std_msgs::Float32) :data))))
                         (setq shoulder-width (* width 1.5))
                         (send *ri* :speak-en (format nil "~Ameter" shoulder-width))
                         t))

  (:set-target-arm ()
                   (when (and (ros::has-param "/target_arm") hand-coords)
                     (return-from :set-target-arm t))
                   (setq pub-bbox nil)
                   (setq fp nil)
                   (send *ri* :speak-jp "手を見せてください" :wait t)
                   (while (null hand-coords)
                     (if is-cancelled
                         (retuen-from :set-target-arm nil)
                         (setq hand-coords (hand-pose-subscriber))))
                   (give-hand-pose hand-coords)
                   (send *ri* :speak-en "set target-arm" :wait t)
                   t)

  (:publish-boundingbox ()
                        (when pub-bbox
                          (retuen-from :publish-boundingbox t))
                        
                        (setq width shoulder_width)
                        (setq arm (ros::get-param "/target_arm"))
                        
                        (setq bbox_req (instance hand_navigation_pr2::SetBBoxPublisherRequest :init))
                        (send bbox_req :switch t)
                        (if (string-equal arm "larm")
                            (send bbox_req :position (float-vector -0.2 (+ 0.5 (/ width 2)) 0.0))
                            (send bbox_req :position (float-vector -0.2 (- -0.5 (/ width 2)) 0.0)))
                        (send bbox_req :dimention (float-vector 0.6 width 2.0))
                        (setq bbox_res (ros::service-call "/boundingbox_publisher/set_param" bbox_req))
                        (send *ri* :speak-en "published bounding box" :wait t)
                        (setq pub-bbox t)
                        t)

  (:resize-footprint()
                    (when fp
                      (retuen-from :resize-footprint t))
                    (setq width shoulder-width)
                    (setq arm target-arm)

                    (if (string-equal arm "larm")
                        (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[-0.5,0.5],[-0.5,~A], [0.1,~A], [0.1,0.5], [0.34,0.35],[0.4,0],[0.34,-0.35]]" (+ 0.5 width) (+ 0.5 width)))
                        (setq footprint (format nil "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35],[0.1,-0.5],[0.1,~A],[-0.5,~A],[-0.5,-0.5]]" (- -0.5 width) (- -0.5 width))))    
                    (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string footprint)
                    (send *ri* :speak-en "changed footprint" :wait t))

  (:reset-footprint()
                   (ros::set-dynamic-reconfigure-param "/move_base_node/local_costmap" "footprint" :string "[[-0.34,-0.35],[-0.34,0.35],[0.34,0.35],[0.4,0],[0.34,-0.35]]")
                   (send *ri* :speak-en "reset footprint" :wait t))
  
  
  (:init ()
         (ros::advertise "/init/result" std_msgs::string 1)
         (ros::subscribe "/init/request" hand_navigation_pr2::InitRequest #'send self :req-cb)
         (ros::subscribe "/init/cancel" hand_navigation_pr2::InitRequest #'send self :cancel-cb))
  
  (:cancel-cb (msg)
              (setq is-cancelled t)
              (send *ri* :cancel-angle-vector))
  
  (:req-cb
   (msg)
   (let ((request (send msg :request))
         (res-msg (instance std_msgs::string :init)))
     (setq is-cancelled nil)
     (case request
       (0
        (when (send self :reset)
          (send res-msg :data "reset")))
       (1
        (when (send self :set-shoulder-width)
          (send res-msg :data "set shoulder-width")))
       (2
        (when (send self :set-target-arm)
          (send res-msg :data "set target-arm")))
       (3
        (when (send self :publish-boundingbox)
          (send res-msg :data "published boundingbox")))
       (4
        (when (send self :resize-footprint)
          (send res-msg :data "resized footprint"))))
          
     (ros::publish "/init/result" res-msg)
     (setq is-cancelled nil))))
  
(setq is (instance init-server :init))
(ros::rate 10)
(do-until-key
    (ros::spin-once)
    (ros::sleep))


