(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "speech_recognition_msgs")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://hand_navigation_pr2/euslisp/get-coords-diff.l")

(ros::roseus "teach_spot")
(pr2-init nil)

(defclass teach-spot
    :slots (mode spot spot-graph spot-list path prev-n goal-n goal-name sol))

(defmethod teach-spot
    (:init ()
     (setq mode :record)
     (setq spot-graph (instance graph :init))
     (setq sol (instance breadth-first-graph-search-solver :init))
     (ros::subscribe "/Tablet/voice" speech_recognition_msgs::SpeechRecognitionCandidates #'send self :voice-cb)
     (ros::subscribe "/amcl_pose" geometry_msgs::posewithcovariancestamped #'send self :spot-cb)
     (ros::ros-info "init ended"))

  (:spot-cb (msg)
            (if msg
                (setq spot (ros::tf-pose->coords (send msg :pose :pose)))))


  (:add-spot (spot-coords &optional name)
             (let (n)
               (setq n (instance node :init (or name (format nil "~A" (send (ros::time-now) :to-sec)))))
               (send n :put :coords spot-coords)
               (ros::ros-info "node ~A ~% coords:~A" (send n :name) (send n :get :coords))
               (dolist (i (send spot-graph :nodes)) ;; if node exists nearby
                 (when (< (abs (get-diff-position spot-coords (send i :get :coords))) 0.2)
                   (send spot-graph :add-arc-from-to i n :both t)
                   (return)))
               (send spot-graph :add-node n)
               (when prev-n
                 (let ((c (abs (get-diff-position spot-coords (send prev-n :get :coords)))))
                   (send (instance costed-arc :init prev-n n 1) :put :name c)
                   (send (instance costed-arc :init n prev-n 1) :put :name c))
                 ;; (send spot-graph :add-arc-from-to prev-n n :both t)
                 (ros::ros-info "arc from ~A to ~A" (send prev-n :name) (send n :name)))
               (setq prev-n n)
               ))

  (:debug-add-nodes ()
                    (dotimes (i 15)
                      (send self :add-spot (make-coords :pos (float-vector i 0 0)) (format nil "~A" i)))
                    )

  (:debug-dot ()
              ;; (send *ri* :speak-jp "グラフをdotファイルで出力します")
              (ros::ros-info "dot")
              (send spot-graph :write-to-dot "a" path))

  (:voice-cb (msg)
       (let ((text (elt (send msg :transcript) 0)))
         (ros::ros-info "~A" text)
         (cond
           ((substringp "に行こう" text)
            (setq goal-name (string-right-trim "に行こう" text))
            (ros::ros-info "goal-name: ~A" goal-name)
            ;; (send *ri* :nod)
            (let ((spot-coords spot) start-n)
              (dolist (n (send spot-graph :nodes)) ;; if node exists nearby
                (when (< (abs (get-diff-position spot-coords (send n :get :coords))) 0.3)
                  (setq start-n n)
                  (return t)))
              (ros::ros-info "start-n: ~A" start-n)
              (when start-n
                (setq goal-n (send spot-graph :node goal-name)) ;; find goal node from graph
                (when goal-n ;; if goal exists in graph
                  (setq path (send sol :solve-by-name spot-graph (send start-n :name) goal-name))
                  (when (path);; if solver returns path from start-n to goal-n
                    ;; (send *ri* :speak-jp "手をつなぎましょう" :wait t)
                    ;;give-hand-pose
                    ;;wait-for-grasp
                    ;; (send *ri* :speak-jp "出発します")
                    (setq spot-list nil)
                    (setq mode :navigation)
                    (return-from :voice-cb)))))

            (ros::ros-info "連れて行ってください")
            (send *ri* :speak-jp "連れて行ってください")
            )

           ((substringp "行き方を教えて" text)
            (setq goal-name (string-right-trim "への行き方を教えて" text))
            (ros::ros-info "goal-name: ~A" goal-name)
            (let ((spot-coords spot) start-n)
              (dolist (n (send spot-graph :nodes)) ;; if node exists nearby
                (when (< (abs (get-diff-position spot-coords (send n :get :coords))) 0.5)
                  (setq start-n n)
                  (return t)))
              (ros::ros-info "start-n: ~A" start-n)
              (when start-n
              (ros::ros-info "1")                
                (setq goal-n (send spot-graph :node goal-name)) ;; find goal node from graph
                (when goal-n ;; if goal exists in graph
                  (ros::ros-info "2")                
                  (setq path (send sol :solve-by-name spot-graph (send start-n :name) goal-name))
                  (when path;; if solver returns path from start-n to goal-n
                                  (ros::ros-info "3")                
                    (dolist (i path)
                      (send *ri* :speak-jp (format nil "~A" (send (send i :state) :name)) :wait t))
                    (return-from :voice-cb))))))

           
         ;; ((substringp "手をつなごう" text)
         ;;  ;;give-hand-pose
         ;;  )
         
         ((and (substringp "ここが" text) (substringp "だよ" text))
          (let (spot-name)
            (setq spot-name (string-left-trim "ここが" (string-right-trim "だよ" text)))
            (ros::ros-info "spot-name: ~A" spot-name)
            (send self :add-spot spot spot-name)
            ;; (send *ri* :speak-jp (format nil "~A" spot-name) :wait t)
            (ros::ros-info (format nil "~Aを覚えました" spot-name))
            (push spot-name spot-list)
            ))
         
         ((string-equal "どこから来ましたか" text)
          (unless (null spot-list)
            (dolist (i spot-list)
              (send *ri* :speak-jp (format nil "~A" i) :wait t))
            (send *ri* :speak-jp "を通りました")))
         
         ((string-equal "ドット" text) (send self :debug-dot))
         (t nil))))

  (:run ()
        (case mode          
          (:record
           (let ((spot-coords spot))
             (unless (null spot)
               (when (null prev-n)
                 (send self :add-spot spot-coords))
               (unless (null prev-n)
                 (when (> (abs (get-diff-position spot-coords (send prev-n :get :coords))) 1.0)
                   (send self :add-spot spot-coords)))
               )))

          (:navigation
           (let* ((next-sn (pop path)) (next-n (send next-sn :state)) (next (send next-n :coords)) (diff (convert-go-to-robot next)) (x (car diff)) (d (cadr diff))) 
             (send *ri* :go-pos-unsafe 0 0 d)
             (send *ri* :go-pos-unsafe x 0 0))
           (when (eq (length path) 0)
             (send *ri* :speak-jp (format nil "~Aに到着しました" goal))
             (setq prev-n goal-n)
             (setq goal-n nil)
             (setq mode :record)
             (setq path nil))))))

(setq *ts* (instance teach-spot :init))
(ros::rate 3)
(do-until-key
 (ros::spin-once)
 (send *ts* :run)
 (ros::sleep))



