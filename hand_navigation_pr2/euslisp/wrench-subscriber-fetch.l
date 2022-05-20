(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://fetcheus/fetch-interface.l")

(ros::roseus "pull_arm")
(fetch-init nil)
(send *ri* :angle-vector #f(22.4077 66.8381 49.9094 -124.795 128.259 -49.9694 46.1586 23.8993 0.050981 1.7321))
(send *ri* :wait-interpolation)
  
(defclass wrench-subscriber
  :slots (force x-list y-list tmp-x tmp-y move-x move-y))

(defmethod wrench-subscriber
  (:init ()
   (ros::subscribe "/endeffector/wrench_transformed" geometry_msgs::WrenchStamped #'send self :wrench-cb)
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
       (unless (> (abs i) 10)
         (return-from :check nil))))
   (when (eq axis :y)
     (dolist (i y-list)
       (unless (> (abs i) 8)
         (return-from :check nil))))
   t)

  (:look-at-hand()
   (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
   (setq end-pos (send *fetch* :rarm :end-coords :worldpos))
   (setq wrench-pos (float-vector (+ (elt end-pos 0) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-x))
                                  (+ (elt end-pos 1) (* (/ 500 (sqrt (+ (expt tmp-x 2) (expt tmp-y 2)))) tmp-y))
                                  1000))                                  
   
   (send *fetch* :head :look-at wrench-pos :revert-if-fail nil)
   ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 900)
   (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 900 :head-controller)
   )
  (:run ()
        (unless (null force)
          (setq tmp-x (+ (send force :x) 12))
          (setq tmp-y (+ (send force :y) 8))
          (setq tmp-z (+ (send force :z) 80))
          (ros::ros-info "x: ~A  y: ~A z: ~A" tmp-x tmp-y tmp-z)

          (setq move-x (send self :check :x))
          (setq move-y (send self :check :y))


	    (send self :look-at-hand)	  
	  (when move-y

	    (when (> tmp-y 10)
	      (send *ri* :send-cmd-vel-raw 0 (/ (log tmp-y) 6.0) :topic-name "/base_controller/cmd_vel"))
	    (when (< tmp-y -10)
	      (send *ri* :send-cmd-vel-raw 0 (- (/ (log (abs tmp-y)) 6.0)) :topic-name "/base_controller/cmd_vel"))

	    (when move-x
	      (when (> tmp-x 16)
		(send *ri* :send-cmd-vel-raw (/ (log tmp-x) 6.50) 0 :topic-name "/base_controller/cmd_vel"))
	      (when (< tmp-x -18)
		(send *ri* :send-cmd-vel-raw (- (/ (log (abs tmp-x)) 10.0)) 0 :topic-name "/base_controller/cmd_vel")))
	    )

	  (when (> (length x-list) 18)
	    (setq x-list (cdr x-list)))
	  (setq x-list (append x-list (list tmp-x)))

	  (when (> (length y-list) 10)
	    (setq y-list (cdr y-list)))
	  (setq y-list (append y-list (list tmp-y))))))

(setq *ws* (instance wrench-subscriber :init))
(ros::rate 80)
(do-until-key
  (ros::spin-once)
  (send *ws* :run)
  (ros::sleep))
         
   
