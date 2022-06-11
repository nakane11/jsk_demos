(defvar *pub-spots-marker-topic* "/taught_spots_marker_array")
(defvar *pub-arcs-marker-topic* "/arcs_marker_array")

(defun make-pin-marker (name coords header id
                        &key (height 1800) (radius 100) (alpha 1.0) (lifetime 15) color)
  "Make pin marker messages for coordinates.
   if :color is set, pin color is set, otherwise random color is set automatically."
  (let ((cyl (make-cylinder radius (* radius 0.7) :segments 9 :pos (float-vector 0 0 (/ height 3))))
        (cub (make-cube (/ radius 3) (/ radius 3) (/ height 2.5) :pos (float-vector 0 0 (/ height 6))))
        (cds (send coords :copy-worldcoords)))

    ;; transform pin
    (send cyl :rotate -pi/2 :y)
    (send cyl :transform cds :world)
    (send cub :transform cds :world)

    ;; generate random color
    (unless color
      (setq color (scale (/ 1.0 255.0) (apply #'float-vector (color-category20 id)))))

    ;; make pin
    (list
     (cylinder->marker-msg cyl header :color color :ns "pin_head" :lifetime lifetime :id id :alpha alpha)
     (cube->marker-msg cub header :color color :ns "pin_body" :lifetime lifetime :id id :alpha alpha)
     (text->marker-msg name (send cds :translate (float-vector 0 0 (/ height 6)))
                       header :ns "pin_label" :lifetime lifetime :id id))))

(defun publish-pin-marker-array (spots header &key (alpha 0.8) (lifetime 15))
  "Publish marker array from spots.
   'spots' is list of cons (name . coords) for each spots."
  (unless (ros::get-topic-publisher *pub-spots-marker-topic*)
    (ros::advertise *pub-spots-marker-topic* visualization_msgs::MarkerArray 5)
    (unix:sleep 1))
  (let ((id 0))
    (ros::publish
     *pub-spots-marker-topic*
     (instance visualization_msgs::MarkerArray :init
               :markers
               (flatten
                (mapcar #'(lambda (spot)
                            (make-pin-marker (car spot) (cdr spot)
                                             header (incf id)
                                             :alpha alpha
                                             :lifetime lifetime)) spots))))))

(defun make-line-list-marker (coords1 coords2 header id
                        &key (alpha 1.0) (lifetime 15) color)
  (let ((li (list (send coords1 :pos) (send coords2 :pos))))

    ;; generate random color
    (unless color
      (setq color (scale (/ 1.0 255.0) (apply #'float-vector (color-category20 id)))))

    ;; make line-list
    (line-list->marker-msg li header :color color :ns "line_list" :lifetime lifetime :id id :alpha alpha)))

(defun publish-arcs-marker-array (nodes header &key (alpha 0.8) (lifetime 15))
  (unless (ros::get-topic-publisher *pub-arcs-marker-topic*)
    (ros::advertise *pub-arcs-marker-topic* visualization_msgs::MarkerArray 5)
    (unix:sleep 1))
  (let (markers (id 0))
    (dolist (n nodes)
      (push (mapcar #'(lambda (i) (make-line-list-marker (send n :get :coords) (send (cdr i) :get :coords) header (incf id) :color #f(0 0 1))) (send n :successors)) markers))
    (ros::publish
     *pub-arcs-marker-topic*
     (instance visualization_msgs::MarkerArray :init
               :markers
               (flatten markers)))))
