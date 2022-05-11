(ros::roseus-add-msgs "geometry_msgs")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "wrench_subscriber")
(pr2-init nil)

(defun force-vector-to-norm(force)
  (expt (+ (expt (send force :x) 2) (expt (send force :y) 2) (expt (send force :z) 2)) 0.5))
  
(defclass wrench-subscriber
  :slots (arm l-force r-force norm x y))

(defmethod wrench-subscriber
  (:init ()
   (ros::subscribe "/left_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :left-cb)
   (ros::subscribe "/right_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :right-cb)
   (setq x 0)
   (setq y 0)
   self)

  (:left-cb (msg)
            (setq l-force (send (send msg :wrench) :force)))

  (:right-cb (msg)
             (setq r-force (send (send msg :wrench) :force)))

  (:run ()
   (when (ros::has-param "/target_arm")
     (setq arm (ros::get-param "/target_arm"))
     (when (string-equal arm "larm")
       (unless (null l-force)
         (when (> (send l-force :y) 20)
           (send *ri* :send-cmd-vel-raw 0 (/ (log (send l-force :y)) 10.0) 0))
         (when (< (send l-force :y) -20)
           (send *ri* :send-cmd-vel-raw 0 (- (/ (log (abs (send l-force :y))) 10.0)) 0))
         (if (> (send l-force :x) 0)
             (setq tmp-x (max (send l-force :x) x))
            (setq tmp-x (min (send l-force :x) x)))
         (when (> tmp-x 30)
           (send *ri* :send-cmd-vel-raw (/ (log x) 15.0) 0 0))
         (when (< tmp-x -20)
           (send *ri* :send-cmd-vel-raw (- (/ (log (abs x)) 10.0)) 0 0))
         ;; (setq x tmp-x)
         ;; (when (or (eq y 0) (> (* (send l-force :y) y) 0)) 
         ;;   (when (> (send l-force :y) 15)
         ;;     (send *ri* :send-cmd-vel-raw 0 (/ (log (send l-force :y)) 10.0) 0))
         ;;   (when (< (send l-force :y) -15)
         ;;     (send *ri* :send-cmd-vel-raw 0 (- (/ (log (abs (send l-force :y))) 10.0)) 0)))

         ;; (when (or (eq x 0) (> (* (send l-force :x) x) 0)) 
         ;;   (when (> (send l-force :x) 25)
         ;;     (send *ri* :send-cmd-vel-raw (/ (log (send l-force :x)) 15.0) 0 0))
         ;;   (when (< (send l-force :x) -15)
         ;;     (send *ri* :send-cmd-vel-raw (- (/ (log (abs (send l-force :x))) 10.0)) 0 0)))
         (setq x (send l-force :x))
         ;; (setq y (send l-force :y))

         
         (setq norm (force-vector-to-norm l-force))
         (ros::ros-info "arm:~A y-force:~A norm:~A~%" arm (send l-force :y) norm)
         (if (> norm 6)
             (ros::set-param "/hand_grasped" t)
           (ros::set-param "/hand_grasped" nil))))
     (when (string-equal arm "rarm")
       (unless (null r-force)
         (when (> (send r-force :y) 10)
           (send *ri* :send-cmd-vel-raw 0 0.2 0))
         (when (< (send r-force :y) -10)
           (send *ri* :send-cmd-vel-raw 0 -0.2 0))
         
         (setq norm (force-vector-to-norm r-force))
         (ros::ros-info "arm:~A y-force:~A norm:~A~%" arm (send r-force :y) norm)
         (if (> norm 6)
             (ros::set-param "/hand_grasped" t)
           (ros::set-param "/hand_grasped" nil)))))))
     
(setq *ws* (instance wrench-subscriber :init))
(ros::rate 40)
(do-until-key
  (ros::spin-once)
  (send *ws* :run)
  (ros::sleep))
         
   
