(ros::roseus-add-msgs "hand_navigation_pr2")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "go")
(pr2-init nil)

(defun cb (server goal)
  (let (msg)
    (if (string-equal (send goal :destination)
    (send *ri* :move-to-wait)
    (setq msg (send server :result :success t))
    (send server :set-succeeded msg)
    ))


(setq go-server
      (instance ros::simple-action-server
                :init "go_action"
                hand_navigation_pr2::GoAction
                :execute-cb cb)) 

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (send go-server :worker)
 (ros::sleep))
