(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://fetcheus/fetch-interface.l")

(ros::roseus "wrench_subscriber")
(fetch-init nil)
(send *ri* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))
  
(defclass wrench-subscriber
  :slots (force x-list y-list tmp-x tmp-y move-x move-y))

(defmethod wrench-subscriber
  (:init ()
   (ros::subscribe "/endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :wrench-cb)
   ;; (ros::subscribe "/joint_states" sensor_msgs::Jointstate #'send self :joint-cb)
   (setq x-list (list 0))
   (setq y-list (list 0))
   self)

  (:wrench-cb
   (msg)
   (setq force (send (send msg :wrench) :force))
   )

  (:check (axis)
   ()
   (when (eq axis :x)
     (dolist (i x-list)
       (unless (> (abs i) 15)
         (return-from :check nil))))
   (when (eq axis :y)
     (dolist (i y-list)
       (unless (> (abs i) 15)
         (return-from :check nil))))
   t)

  (:rad-to-vel (rad)
               (* 0.35 (tan (/ (- rad (/ pi 4)) 1.1))))

  ;; (:look-at-hand
  ;;  (arm)
  ;;  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  ;;  (setq end-pos (send *fetch* arm :end-coords :worldpos))
  ;;  (setq wrench-pos (float-vector (+ (elt end-pos 0) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-x))
  ;;                                 (+ (elt end-pos 1) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-y))
  ;;                                 1400))
   ;; (setq wrench-pos (float-vector (+ (elt end-pos 0) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2) (expt tmp-z 2)))) tmp-x))
   ;;                                (+ (elt end-pos 1) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2) (expt tmp-z 2)))) tmp-y))
   ;;                                (+ (elt end-pos 2) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2) (expt tmp-z 2)))) tmp-z))))
                                  
   
   ;; (send *fetch* :head :look-at wrench-pos)
   ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 900))
       
  (:run ()
        (unless (null force)
          (setq tmp-x (+ (send force :x) 12))
          (setq tmp-y (+ (send force :y) 8))
          (setq tmp-z (+ (send force :z) 80))
          ;; (ros::ros-info "x: ~A  y: ~A z: ~A" tmp-x tmp-y tmp-z)

          (setq move-x (send self :check :x))
          (setq move-y (send self :check :y))
          
          (block send-cmd-vel
            ;; (if (string-equal arm "larm")
            ;;     (send self :look-at-hand :larm)
            ;;     (send self :look-at-hand :rarm)))
            
            (when move-y
              (when (> tmp-y 10)
                (send *ri* :send-cmd-vel-raw 0 (/ (log tmp-y) 12.0) ))
              (when (< tmp-y -10)
                (send *ri* :send-cmd-vel-raw 0 (- (/ (log (abs tmp-y)) 12.0)) ))

              (when move-x
                (when (> tmp-x 10)
                  (send *ri* :send-cmd-vel-raw (/ (log tmp-x) 8.0) 0 ))
                (when (< tmp-x -10)
                  (send *ri* :send-cmd-vel-raw (- (/ (log (abs tmp-x)) 8.0)) 0)))
              )

            (when (> (length x-list) 10)
              (setq x-list (cdr x-list)))
            (setq x-list (append x-list (list tmp-x)))

            (when (> (length y-list) 10)
              (setq y-list (cdr y-list)))
            (setq y-list (append y-list (list tmp-y)))))))

  
  ;;     (setq norm (force-vector-to-norm l-force))
  ;;     (ros::ros-info "arm:~A y-force:~A norm:~A~%" arm (send l-force :y) norm)
  ;;     (if (> norm 6)
  ;;         (ros::set-param "/hand_grasped" t)
  ;;       (ros::set-param "/hand_grasped" nil))))
     
(setq *ws* (instance wrench-subscriber :init))
(ros::rate 80)
(do-until-key
  (ros::spin-once)
  (send *ws* :run)
  (ros::sleep))
         
   
