(ros::roseus-add-msgs "hand_navigation_pr2")
(ros::roseus "hand_navigation_motion_server")

(defun execute-cb (server goal)
  (let (msg)
    (print (send goal :goal))

    (dotimes (i 10)
      (setq msg (send server :feedback :status (format nil "~A" i)))
      (send server :publish-feedback msg)
      (unix::sleep 1)
      )

    (setq msg (send server :result :success t))
    (send server :set-succeeded msg)
    ))

(setq hand-navigation-motion-server
      (instance ros::simple-action-server
                :init "hand_navigation_motion"
                hand_navigation_pr2::HandNavigationAction
                :execute-cb #'execute-cb))
(ros::rate 10)
(do-until-key
    ;; (ros::ros-info "running~%")
 (ros::spin-once)
 (send hand-navigation-motion-server :worker)
 (ros::sleep))

