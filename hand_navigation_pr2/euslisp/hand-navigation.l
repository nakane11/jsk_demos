(ros::roseus-add-msgs "hand_navigation_pr2")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "hand_navigation")
(pr2-init nil)

(when (not (boundp '*init-client*))
  (setq *init-client* (instance ros::simple-action-client :init
				"give_hand_init" hand_navigation_pr2::GiveHandInitAction)))
(send *init-client* :wait-for-server)

(defun execute-cb (server goal)
  (let (msg (send goal :goal :transcript))
    ;; (print (send goal :goal))

    ;; (dotimes (i (send goal :goal :task_id))
    ;;   (setq msg (send server :feedback :status (format nil "~A" i)))
    ;;   (send server :publish-feedback msg)
    ;;   (unix::sleep 1)
    ;;   )

    (cond ((string-equal msg "")
	   (setq goal-action (instance hand_navigation_pr2::GiveHandInitGoal :init))
	   (send goal-action :task_id 1)
	   (send *init-client* :send-goal goal-action)
	   (send *init-client* :wait-for-result)))
          
    (setq msg (send server :result :success t))
    (send server :set-succeeded msg)))

(setq hand-navigation-server
      (instance ros::simple-action-server
                :init "hand_navigation"
                hand_navigation_pr2::SpeechAction
                :execute-cb #'execute-cb))

(ros::rate 10)
(do-until-key
    ;; (ros::ros-info "running~%") 
 (ros::spin-once)
 (send hand-navigation-server :worker)
 (ros::sleep))

