(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "wrench_subscriber")
(pr2-init nil)

(defun force-vector-to-norm(force)
  (expt (+ (expt (send force :x) 2) (expt (send force :y) 2) (expt (send force :z) 2)) 0.5))
  
(defclass wrench-subscriber
  :slots (arm l-force r-force x-list y-list l-shoulder-pan r-shoulder-pan))

(defmethod wrench-subscriber
  (:init ()
   (ros::subscribe "/left_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :left-cb)
   (ros::subscribe "/right_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :right-cb)
   (ros::subscribe "/joint_states" sensor_msgs::Jointstate #'send self :joint-cb)
   (setq x-list nil)
   (setq y-list nil)
   self)

  (:left-cb
   (msg)
   (setq l-force (send (send msg :wrench) :force)))

  (:right-cb
   (msg)
   (setq r-force (send (send msg :wrench) :force)))

  (:joint-cb
   (msg)
   (setq l-shoulder-pan (elt (send msg :position) 32))
   (setq r-shoulder-pan (elt (send msg :position) 18)))


  (:check (axis)
   ()
   (when (eq axis :x)
     (dolist (i x-list)
       (unless (> (abs i) 15)
         (return-from :check nil))))
   (when (eq axis :y)
     (dolist (i y-list)
       (unless (> (abs i) 8)
         (return-from :check nil))))
   t)

  (:rad-to-vel (rad)
               (* 0.1 (tan (/ (rad - (/ pi 4)) 1.1))))

  (:run ()
   (when (ros::has-param "/target_arm")
     (setq arm (ros::get-param "/target_arm"))
     (cond
      ((and (string-equal arm "larm") (not (null l-force)) (not (null l-shoulder-pan)))
       (progn
         (if (or (< l-shoulder-pan 0) (> l-shoulder-pan 1.57))
             (setq rotate (/ l-shoulder-pan pi))
           (setq rotate nil))
         (setq tmp-x (send l-force :x))
         (setq tmp-y (send l-force :y))
         (setq tmp-z (send l-force :z))))

      ((and (string-equal arm "rarm") (not (null r-force)) (not (null r-shoulder-pan)))
       (progn
         (if ((or (< r-shoulder-pan -1.57) (> r-shoulder-pan 0)))
             (setq rotate (- (self :rad-to-vel (abs r-shoulder-pan))))
           (setq rotate nil))
         (setq tmp-x (send r-force :x))
         (setq tmp-y (send r-force :y))
         (setq tmp-z (send r-force :z))))
      (t (return-from :run nil)))

     (ros::ros-info "x: ~A  y: ~A z: ~A" tmp-x tmp-y tmp-z)

     ;; (unless (null rotate)
     ;;   (send *ri* :send-cmd-vel-raw 0 0 rotate)
     ;;   (setq rotate nil)
     ;;   (return-from :run))
     
     ;; (when (send self :check :y)
     ;;   (when (> tmp-y 10)
     ;;     (send *ri* :send-cmd-vel-raw 0 (/ (log tmp-y) 8.0) 0))
     ;;   (when (< tmp-y -10)
     ;;     (send *ri* :send-cmd-vel-raw 0 (- (/ (log (abs tmp-y)) 8.0)) 0)))

     ;; (when (send self :check :x)
     ;;   (when (> tmp-x 18)
     ;;     (send *ri* :send-cmd-vel-raw (/ (log tmp-x) 6.0) 0 0))
     ;;   (when (< tmp-x -12)
     ;;     (send *ri* :send-cmd-vel-raw (- (/ (log (abs tmp-x)) 6.0)) 0 0)))

     (when (> (length x-list) 10)
       (setq x-list (cdr x-list)))
     (setq x-list (append x-list (list tmp-x)))

     (when (> (length y-list) 10)
       (setq y-list (cdr y-list)))
     (setq y-list (append y-list (list tmp-y))))))

         
     ;;     (setq norm (force-vector-to-norm l-force))
     ;;     (ros::ros-info "arm:~A y-force:~A norm:~A~%" arm (send l-force :y) norm)
     ;;     (if (> norm 6)
     ;;         (ros::set-param "/hand_grasped" t)
     ;;       (ros::set-param "/hand_grasped" nil))))
     
(setq *ws* (instance wrench-subscriber :init))
(ros::rate 50)
(do-until-key
  (ros::spin-once)
  (send *ws* :run)
  (ros::sleep))
         
   
