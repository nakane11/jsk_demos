(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(ros::roseus "human-filter")
(ros::advertise "human_filtered" sensor_msgs::PointCloud2 1)

(setq *tfl* (instance ros::transform-listener :init))

(defun people-array-subscriber(people-pose-array)
  (let ((people-poses (send people-pose-array :poses)))
    (setq ret nil)
    (setq stamp (send people-pose-array :header :stamp))
    (setq base-to-camera-coords (send *tfl* :lookup-transform "base_footprint" (send people-pose-array :header :frame_id) stamp))
    (when base-to-camera-coords
      (when (> (length people-poses) 0)
        (dolist (people-pose people-poses)
          (setq poses (send people-pose :poses))
          (setq limb-names (send people-pose :limb_names))
          (when (> (length poses) 0)
            (dolist (pose poses)
              ;;
              (setq camera-to-point-coords (ros::tf-pose->coords pose))
              (setq base-to-point (send base-to-camera-coords :transform camera-to-point-coords))
              (push base-to-point ret)))))))
  ret)
            
(defun pointcloud2-subscriber(pc2 people-coords-list)
  (let ((width (send pc2 :width)) (points (send pc2 :data)))
    (setq pointcloud-msg (instance sensor_msgs::PointCloud2 :init))
    (when (> width 0)
      (dotimes (i width)
        (setq point-data (subseq points (* 16 i) (* 16 (+ 1 i))))
        (setq point-xyz (data-to-xyz point-data))
        (dolist (people-coords people-coords-list)
          (when (or (< 500 (abs (- (elt point-xyz 0) (elt (send people-coords :position) 0))))
                    (< 500 (abs (- (elt point-xyz 1) (elt (send people-coords :position) 1)))))
            ;; (setq (send pointcloud-msg :data) (append (send pointcloud-msg :data)))
            (setq (send pointcloud-msg :data)
                  (format nil "~A~A" (send pointcloud-msg :data) point-data))
            (return) ;escape dolist
            ;;
            )
          )))))


(defun struct (intval) (let ((str (make-string 4))) (sys::poke intval str 0 :integer) (sys::peek str 0 :float)))

(defun convert-float32(list)
    (setq x (+ (* (car list) (expt 2 24)) (* (cadr list) (expt 2 16)) (* (caddr list) (expt 2 8)) (cadddr list)))
    (struct x))

(defun data-to-xyz(point-data)
   (let ((x-list (subseq point-data 0 4))
         (y-list (subseq point-data 4 8))
         (z-list (subseq point-data 8 12)))
     (list (convert-float32 x-list) (convert-float32 y-list) (convert-float32 z-list))))


(defclass human-filter
    :super exact-time-message-filter)

(defmethod human-filter
    (:callback (pc2 poses)
     (setq people-coords (people-array-subscriber poses))
     (setq pub (pointcloud2-subscriber pc2 people-coords))
     (ros::publish "human_filtered" pub)))

(setq *mf* (instance human-filter :init
                     (list (list "/base_scan_filtered" sensor_msgs::PointCloud2)
                           (list "/edgetpu_human_pose_estimator/output/poses" jsk_recognition_msgs::PeoplePoseArray))))

(ros::spin)
(exit)
