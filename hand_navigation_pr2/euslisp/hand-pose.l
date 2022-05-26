(ros::roseus-add-msgs "hand_navigation_pr2")
(load "package://pr2eus/pr2-interface.l")

(defclass hand-pose
    :slots (server servo-client grasp-client servo-goal))

(defmethod hand-pose
    (:wait-for-grasp()
     (let is_grasped
       (while (null is_grasped)
         (send grasp-client :send-goal (instance hand_navigation_pr2::GraspGoal :init))
         (setq is_grasped (send (send grasp-client :get-result) :grasped)))))

    (:wait-for-ungrasp()
     (let ((is_grasped t))
       (while (is_grasped)
         (send grasp-client :send-goal (instance hand_navigation_pr2::GraspGoal :init))
         (setq is_grasped (send (send grasp-client :get-result) :grasped)))))
  
  
  (:default-hand ()
      (send servo-goal :goal :angle_array (float-vector 90 89 87 90 0))
    (send servo-client :send-goal servo-goal)
    (send servo-client :wait-for-result))
  
  (:hold-hand ()
              (send servo-goal :goal :angle_array (float-vector 71 82 96 119 0))              
              (send servo-client :send-goal servo-goal)
              (send self :wait-for-grasp)
              (send servo-goal :goal :angle_array (float-vector 83 91 89 104 140))              
              (send servo-client :send-goal servo-goal)
              (send servo-client :wait-for-result)
              (send *pr2* :head :angle-vector #f(0.0 12))
              (send *ri* :angle-vector (send *pr2* :angle-vector) 800))

  (:release-hand ()
              (send servo-goal :goal :angle_array (float-vector 71 82 96 119 0))              
              (send servo-client :send-goal servo-goal)
              (send self :wait-for-ungrasp)
              (send servo-goal :goal :angle_array (float-vector 90 89 87 90 0))
              (send servo-client :send-goal servo-goal)
              (send servo-client :wait-for-result)
              (send *pr2* :head :angle-vector #f(0.0 12))
              (send *ri* :angle-vector (send *pr2* :angle-vector) 800))
  
  (:execute-cb (server hand-pose-goal)
               (let ((msg (send server :result)) (pose (send hand-pose-goal :goal :pose)))
                 (case pose
                   (0 (send self :default-hand))
                   (1 (send self :hold-hand))
                   (2 (send self :release-hand)))
                 (send server :set-succeeded msg))
     )


    (:init ()
           (setq servo-goal (instance hand_navigation_pr2::ServoActionGoal :init))
           (setq server (instance ros::simple-action-server :init
                                  "hand_pose" hand_navigation_pr2::HandPoseAction
                                  :execute-cb `(lambda-closure nil 0 0 (server goal)
                                                               (send ,self :execute-cb server goal))))
           (setq servo-client (instance ros::simple-action-client :init
                                  "servo_interpolation" hand_navigation_pr2::ServoAction))
           (send servo-client :wait-for-server)
           (setq grasp-client (instance ros::simple-action-client :init
                                  "grasp" hand_navigation_pr2::GraspAction))
           (send grasp-client :wait-for-server))

  (:run ()
        (ros::rate 10)        
        (do-until-key
            (ros::spin-once)
          (send server :worker)
          (ros::sleep)))
)

(ros::roseus "hand_pose")
(pr2-init nil)
(setq hand (instance hand-pose :init))
(send hand :run)
