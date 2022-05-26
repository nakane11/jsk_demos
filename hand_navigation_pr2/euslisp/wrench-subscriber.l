(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "wrench_subscriber")
(pr2-init nil)
  
(defclass wrench-subscriber
  :slots (arm tmp-x tmp-y tmp-z l-force r-force x-list y-list l-shoulder-pan r-shoulder-pan move-x move-y))

(defmethod wrench-subscriber
  (:init ()
   (ros::subscribe "/left_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :left-cb)
   (ros::subscribe "/right_endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :right-cb)
   (ros::subscribe "/joint_states" sensor_msgs::Jointstate #'send self :joint-cb)
   (setq x-list (list 0))
   (setq y-list (list 0))
   self)

  (:left-cb
   (msg)
   (setq l-force (send (send msg :wrench) :force))
   )

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
               (* 0.35 (tan (/ (- rad (/ pi 4)) 1.1))))

  ;; (:look-at-hand(arm)
  ;;  (let ((pan (rad2deg (atan tmp-y tmp-x))) (tilt (+ 25 (* 2 tmp-z))))
  ;;    (if (> pan 0)
  ;;        (setq pan (min pan 168))
  ;;        (setq pan (max pan -168)))
  ;;    (if (> tilt 0)
  ;;        (setq tilt (min tilt 60))
  ;;        (setq tilt (max tilt -30)))
  ;;    (send *pr2* :head :angle-vector (float-vector pan tilt)))
 
  (:look-at-hand
   (arm)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (setq end-pos (send *pr2* arm :end-coords :worldpos))
   (setq wrench-pos (float-vector (+ (elt end-pos 0) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-x))
                                  (+ (elt end-pos 1) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-y))
                                  1400))
   (send *pr2* :head :look-at wrench-pos)
   (send *ri* :angle-vector (send *pr2* :angle-vector) 500 :head-controller))
       
  (:run ()
   (when (ros::has-param "/target_arm")
     (setq arm (ros::get-param "/target_arm"))

     (cond
      ((and (string-equal arm "larm") (not (null l-force)) (not (null l-shoulder-pan)))
       (progn
         (if (or (< l-shoulder-pan 0.2) (> l-shoulder-pan 1.37))
             (setq rotate (send self :rad-to-vel l-shoulder-pan))
           (setq rotate nil))
         (setq tmp-x (send l-force :x))
         (setq tmp-y (send l-force :y))
         (setq tmp-z (send l-force :z))))

      ((and (string-equal arm "rarm") (not (null r-force)) (not (null r-shoulder-pan)))
       (progn
         (if (or (< r-shoulder-pan -1.37) (> r-shoulder-pan -0.2))
             (setq rotate (send self :rad-to-vel r-shoulder-pan))
           (setq rotate nil))
         (setq tmp-x (send r-force :x))
         (setq tmp-y (send r-force :y))
         (setq tmp-z (send r-force :z))))
      (t (return-from :run nil)))

     (ros::ros-info "x: ~A  y: ~A z: ~A" tmp-x tmp-y tmp-z)

     (setq move-x (send self :check :x))
     (setq move-y (send self :check :y))

     ;; (if (string-equal arm "larm")
     ;;     (send self :look-at-hand :larm)
     ;;     (send self :look-at-hand :rarm))
     
     (block send-cmd-vel
            (unless (null rotate)
              (send *ri* :send-cmd-vel-raw 0 0 rotate)
              (return-from send-cmd-vel))
            
            (when move-y
              (when (> tmp-y 10)
                (send *ri* :send-cmd-vel-raw 0 (/ (log tmp-y) 8.0) 0))
              (when (< tmp-y -10)
                (send *ri* :send-cmd-vel-raw 0 (- (/ (log (abs tmp-y)) 8.0)) 0)))

            (when move-x
              (when (> tmp-x 18)
                (send *ri* :send-cmd-vel-raw (/ (log tmp-x) 6.0) 0 0))
              (when (< tmp-x -15)
                (send *ri* :send-cmd-vel-raw (- (/ (log (abs tmp-x)) 6.0)) 0 0))))

     (when (> (length x-list) 10)
       (setq x-list (cdr x-list)))
     (setq x-list (append x-list (list tmp-x)))

     (when (> (length y-list) 10)
       (setq y-list (cdr y-list)))
     (setq y-list (append y-list (list tmp-y))))))

(setq *ws* (instance wrench-subscriber :init))
(ros::rate 50)
(do-until-key
 (ros::spin-once)
 (send *ws* :run)
 (ros::sleep))


