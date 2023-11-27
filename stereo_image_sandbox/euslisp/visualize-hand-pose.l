;;;
;;;	roslaunch realsense2_camera rs_rgbd.launch gui:=false
;;;	roslaunch stereo_image_sandbox hand_pose_estimation_d405.launch gui:=false
;;;

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "listener")

(defvar *topic-name* "/camera/color/hand_pose_estimation/output/skeleton")
(defvar *skeleton-array* nil)

(defun skeleton->cube (skeleton)
  (let* (cube-a
         cube-b
         bone
         cube-list)
    (dotimes (i (length (send skeleton :bones)))
      (setq bone (elt (send skeleton :bones) i))
      (setq cube-a (make-cube 10 10 10))
      (setq cube-b (make-cube 10 10 10))
      (send cube-a :newcoords (make-coords :pos (ros::tf-point->pos (send bone :start_point))))
      (send cube-b :newcoords (make-coords :pos (ros::tf-point->pos (send bone :end_point))))
      (setq cube-list (append cube-list (list cube-a cube-b))))
    cube-list))


(defun human-skeleton-array-cb (msg)
  (setq *skeleton-array* (send msg :skeletons))
  (when *skeleton-array*
    (send *irtviewer* :draw-objects :flush nil)
    (dolist (cube-list (mapcar #'skeleton->cube *skeleton-array*))
      (dolist (cube cube-list)
        (print (send cube :worldpos))
        (send cube :draw-on :flush nil :color #f(1 0 0)))))
  (send *irtviewer* :viewer :viewsurface :flush))

(ros::subscribe *topic-name* jsk_recognition_msgs::HumanSkeletonArray #'human-skeleton-array-cb 1)

(unless (boundp '*irtviewer*) (make-irtviewer))

(do-until-key
    (x::window-main-one)
  (ros::spin-once)
  (ros::sleep)
  )
