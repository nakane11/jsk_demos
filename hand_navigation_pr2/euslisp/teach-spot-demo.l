(ros::roseus-add-msgs "speech_recognition_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(load "package://pr2eus/pr2-interface.l")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://hand_navigation_pr2/euslisp/get-coords-diff.l")
(ros::roseus "teach_spot")
(pr2-init nil)

(defun my-string-left-trim (bag str)
  (declare (string str))
  (declare (string bag))
  (if (string= (subseq str 0 (length bag)) bag)
      (subseq str (length bag))
      nil))

(defun my-string-right-trim (bag str)
  (declare (string str))
  (declare (string bag))
  (if (string= (subseq str (- (length str) (length bag))) bag)
      (subseq str 0 (- (length str) (length bag)))
      nil))

(defclass teach-spot
    :slots (label mode spot spot-graph spot-list path prev-n goal-n goal-name sol))

(defmethod teach-spot
    (:init ()
     (setq label 0)
     (setq mode :record)
     (setq spot-graph (instance graph :init))
     (setq sol (instance breadth-first-graph-search-solver :init))
     (ros::subscribe "/Tablet/voice" speech_recognition_msgs::SpeechRecognitionCandidates #'send self :voice-cb)
     (ros::subscribe "/amcl_pose" geometry_msgs::posewithcovariancestamped #'send self :spot-cb)
     (ros::advertise "/follow_waypoints/initialpose" geometry_msgs::posewithcovariancestamped 1)
     (ros::advertise "/path_reset" std_msgs::empty 1)
     (ros::advertise "/path_ready" std_msgs::empty 1)
     (ros::ros-info "init ended"))

  (:spot-cb (msg)
            (if msg
                (setq spot (ros::tf-pose->coords (send msg :pose :pose)))))


  (:add-spot (spot-coords &optional name)
             (when (null spot-coords)
               (return-from :add-spot))
             (dolist (i (send spot-graph :nodes)) ;; if node exists nearby, connect to prev-n instead of adding new one 
               (when (< (abs (get-diff-position spot-coords (send i :get :coords))) 0.4)
                 (unless (and (member (send i :name) spot-list :test #'string-equal) name)
                   (when name
                     (send i :name name))
                   (send spot-graph :add-arc-from-to prev-n i :both t)
                   (setq prev-n i)
                   (return-from :add-spot))))

             (let (n)
               (setq n (instance node :init (or name (format nil "~A" label))))
               (unless name (incf label))
               (send n :put :coords spot-coords)
               (ros::ros-info "node ~A ~% coords:~A" (send n :name) (send n :get :coords))
               (send spot-graph :add-node n)

               (when prev-n
                 (send spot-graph :add-arc-from-to prev-n n :both t)
                 (ros::ros-info "arc from ~A to ~A" (send prev-n :name) (send n :name)))
               ;; (when prev-n
               ;;   (let ((c (abs (get-diff-position spot-coords (send prev-n :get :coords)))))
               ;;     (send (instance costed-arc :init prev-n n 1) :put :name c)
               ;;     (send (instance costed-arc :init n prev-n 1) :put :name c)

                   ;; (send *ri* :speak-jp (format nil "~Aから~A ~A" (send prev-n :name) (send n :name) c) :wait t)
               (setq prev-n n)))

  (:output-graph ()
              ;; (send *ri* :speak-jp "グラフを出力します")
              (ros::ros-info "graph")
              (send spot-graph :write-to-file (format nil "~A" (send (ros::time-now) :to-nsec) path "output" "png")))

  (:voice-cb (msg)
             (let ((text (elt (send msg :transcript) 0)))
               (ros::ros-info "text:~A" text)
               
               (cond
                 ((substringp "に行こう" text)
                  (setq goal-name (my-string-right-trim "に行こう" text))
                  (setq goal-n (send spot-graph :node goal-name))
                  (ros::ros-info "goal-node: ~A" (send goal-n :name))
                  (when goal-n
                    (let ((spot-coords spot) start-n)
                      (dolist (n (send spot-graph :nodes)) ;; if node exists nearby
                        (when (< (abs (get-diff-position spot-coords (send n :get :coords))) 0.4)
                          (setq start-n n)
                          (return)))
                      (ros::ros-info "start-node: ~A" (send start-n :name))
                      (when start-n
                        (setq path nil)
                        (setq path (send sol :solve-by-name spot-graph (send start-n :name) goal-name))
                        (when path 
                          ;; (setq mode :navigation)
                          (ros::publish "/path_reset" (instance std_msgs::empty :init))
                          (dolist (i path)
                            (let ((pose (instance geometry_msgs::posewithcovariancestamped :init)))
                              (send (send pose :header) :frame_id "map")
                              (send (send pose :pose) :pose (ros::coords->tf-pose (send (send i :state) :get :coords)))
                              (ros::publish "/follow_waypoints/initialpose" pose)))
                          (ros::publish "/path_ready" (instance std_msgs::empty :init))
                          (return-from :voice-cb)))))

                  (ros::ros-info "solver failed")
                  ;; (send *ri* :speak-jp "連れて行ってください")
                  )

                 ((substringp "の行き方を教えて" text)
                  (setq goal-name (my-string-right-trim "の行き方を教えて" text))
                  (setq goal-n (send spot-graph :node goal-name))
                  (ros::ros-info "goal-node: ~A" (send goal-n :name))
                  (when goal-n
                    (let ((spot-coords spot) start-n)
                      (dolist (n (send spot-graph :nodes))
                        (when (< (abs (get-diff-position spot-coords (send n :get :coords))) 0.4)
                          (setq start-n n)
                          (return)))
                      (ros::ros-info "start-node: ~A" (send start-n :name))
                      (when start-n
                        (setq path nil)
                        (setq path (send sol :solve-by-name spot-graph (send start-n :name) goal-name))
                        (when path
                          (dolist (i path)
                            (ros::ros-info (format nil "~A" (send (send i :state) :name))))
                          ;; (send *ri* :speak-jp "の順に通ります" :wait t)
                          (return-from :voice-cb))))
                    (ros::ros-info "solver failed")))
                 
                 ((and (substringp "ここが" text) (substringp "だよ" text))
                  (let (spot-name (spot-coords spot))
                    (setq spot-name (my-string-left-trim "ここが" (my-string-right-trim "だよ" text)))
                    (unless (null spot-coords)
                      (ros::ros-info "spot-name: ~A" spot-name)
                      (push spot-name spot-list)
                      (unless (null prev-n)
                        (when (< (abs (get-diff-position spot-coords (send prev-n :get :coords))) 0.4)
                          (unless (member (send prev-n :name) spot-list :test #'string-equal)
                            (ros::ros-info (format nil "rename ~A to ~A" (send prev-n :name) spot-name))
                            (send prev-n :put :coords spot-coords)
                            (send prev-n :name spot-name)
                            (return-from :voice-cb))))
                      (send self :add-spot spot spot-name))
                    ))
                 
                 ((string-equal "どこに行けますか" text)
                  (unless (null spot-list)
                    (dolist (i spot-list)
                      (send *ri* :speak-jp (format nil "~A" i) :wait t))
                    (send *ri* :speak-jp "に行けます")))
                 
                 ((string-equal "グラフを出して" text) (send self :output-graph))
                 (t nil))))

  (:run ()
        (case mode          
          (:record
           (let ((spot-coords spot))
             (unless (null spot)
               (when (null prev-n)
                 (send self :add-spot spot-coords))
               (unless (null prev-n)
                 (when (> (abs (get-diff-position spot-coords (send prev-n :get :coords))) 0.6)
                   (send self :add-spot spot-coords)))
               )))

          (:navigation
           (ros::ros-info "navigation")
           (while (length path)
             (let* ((next-sn (pop path)) (next-n (send next-sn :state)) (next (send next-n :get :coords)) (diff (convert-go-to-robot next)) (x (car diff)) (d (cadr diff)))
               (ros::ros-info "x:~A d:~A" x d)
               (send *ri* :go-pos 0 0 d)
               (send *ri* :go-pos x 0 0)))
           ;; (send *ri* :speak-jp (format nil "~Aに到着しました" goal))
           (setq prev-n goal-n)
           (setq goal-n nil)
           (setq mode :record)
           (setq path nil)))))

(setq *ts* (instance teach-spot :init))
(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (send *ts* :run)
 (ros::sleep))
