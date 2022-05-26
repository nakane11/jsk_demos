(ros::roseus-add-msgs "hand_navigation_pr2")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")

(defclass grasp-detector
    :slots (hand-list server))

(defmethod grasp-detector
    (:execute-cb (server goal)
     (let (arm (msg (send server :result :result nil)) (dist-list nil) (count 0))
       (unless (ros::has-param "/target_arm")
         (send server :set-succeeded msg)
         (return-from :execute-cb))
       (setq arm (ros::get-param "/target_arm"))
       (cond
         ((string= arm "larm") (setq arm :larm))
         ((string= arm "rarm") (setq arm :rarm))
         (t (send server :set-succeeded msg)
            (return-from :execute-cb)))
       (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
       (send *pr2* :head :look-at (send *pr2* arm :end-coords :worldpos))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 500 :head-controller)
       (dotimes (i 5)
         (let ((hand-poses (one-shot-subscribe "/hand_pose_estimation_2d/output/pose" jsk_recognition_msgs::HandPoseArray)) hand-pose finger-pose stamp base-to-camera-coords camera-to-hand-coords human-hand-coords robot-hand-coords (dist nil))
           (when (> (length (send hand-poses :poses)) 0)
             (setq hand-pose (car (send hand-poses :poses)))
             (when (> (length (send hand-pose :poses)) 0) 
              (setq finger-pose (car (send hand-pose :poses)))
               (setq stamp (send hand-poses :header :stamp))
               (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_footprint" (send hand-poses :header :frame_id) stamp))
               (when base-to-camera-coords
                 (setq camera-to-hand-coords (ros::tf-pose->coords finger-pose))
                 (setq human-hand-coords (send base-to-camera-coords :transform camera-to-hand-coords))
                 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
                 (setq robot-hand-coords (send *pr2* arm :end-coords))
                 (setq dist (norm (send (send human-hand-coords :transformation robot-hand-coords) :worldpos))))))
           (push dist dist-list)
           (ros::ros-info "~A" dist)))
       (dolist (dist dist-list)
         (unless (or (null dist) (> dist 160))
           (incf count)))
       (if (> count 2)
           (progn
             (send msg :result :grasped t)
             (send *ri* :speak-en "grasp")) 
           (send msg :result :grasped nil))
       (send msg :result :result t)
       (send server :set-succeeded msg)))
  (:init ()
         (setq hand-list nil) 
         (setq server (instance ros::simple-action-server :init
                                "grasp" hand_navigation_pr2::GraspAction
                                :execute-cb `(lambda-closure nil 0 0 (server goal)
                                                             (send ,self :execute-cb server goal)))))
  (:run ()
        (ros::rate 5)        
        (do-until-key
            (ros::spin-once)
          (send server :worker)
          (ros::sleep)))
)

(ros::roseus "grasp_detector")
(pr2-init nil)
(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(setq grasp (instance grasp-detector :init))
(send grasp :run)


