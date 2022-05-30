(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "speech_recognition_msgs")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://hand_navigation_pr2/euslisp/get-coords-diff.l")

(ros::roseus "teach_spot")
(pr2-init nil)

(defclass teach-spot
    :slots (mode spot-graph path prev-n goal-name sol))

(defmethod teach-spot
    (:init ()
     (setq mode :idling)
     (setq spot-graph (instance graph :init))
     (setq sol (instance breadth-first-graph-search-solver :init))
     (ros::subscribe "/Tablet/voice" speech_recognition_msgs::SpeechRecognitionCandidates #'send self :voice-cb)
     (ros::ros-info "init ended"))

  (:add-spot (spot-coords &optional name)
             (ros::ros-info "add-spot")
             (let (n)
               (setq n (instance node :init (or name (format nil "~A" (ros::time-now)))))
               (send n :put :coords spot-coords)
               (send spot-graph :add-node n)
               (ros::ros-info "node ~A ~% coords:~A" (send n :name) (send n :get :coords))
               (when prev-n
                 (send spot-graph :add-arc-from-to prev-n n :both t)
                 (ros::ros-info "arc from ~A to ~A" (send prev-n :name) (send n :name)))
               (setq prev-n n)
               ))

  (:debug-add-nodes ()
                    (dotimes (i 15)
                      (send self :add-spot (make-coords :pos (float-vector i 0 0)) (format nil "~A" i)))
                    (send spot-graph :write-to-pdf "test")
                    )

  (:voice-cb (msg)
       (let ((text (elt (send msg :transcript) 0)))
         (ros::ros-info "~A" text)
         (cond
           ((substringp "に行こう" text)
            (setq goal-name (string-right-trim "に行こう" text))
            (ros::ros-info "goal-name: ~A" goal-name)
            ;; (send *ri* :nod)
            (let ((spot-coords (save-spot)) start-n goal-n)
              (dolist (n (send spot-graph :nodes)) ;; if node exists nearby
                (when (< (abs (get-diff-position spot-coords (send n :get :coords))) 0.8)
                  (set start-n n)
                  (return t)))
              (when start-n
                (setq goal-n (send spot-graph :node goal-name)) ;; find goal node from graph
                (when goal-n ;; if goal exists in graph
                  (setq path (send sol :solve-by-name spot-graph (send start-n :name) goal-name))
                  (when (path);; if solver returns path from start-n to goal-n
                    (send *ri* :speak-jp "手をつなぎましょう" :wait t)
                    ;;give-hand-pose
                    ;;wait-for-grasp
                    (send *ri* :speak-jp "出発します")
                    (setq mode :navigation)
                    (return-from :voice-cb)))
                (setq prev-n start-n)));; link existing node and starting point
            ;; else entering record mode
            (ros::ros-info "連れて行ってください")
            ;; (send *ri* :speak-jp "連れて行ってください")
            )
         
         ;; ((substringp "手をつなごう" text)
         ;;  ;;give-hand-pose
         ;;  )
         
         ((substringp "道順を覚えて" text)
          ;; (send *ri* :speak-jp "はい")
          (ros::ros-info "はい")
          ;; (send *ri* :nod)      
          (send self :add-spot (save-spot))
          (setq mode :record))

         ((and (substringp "ここが" text) (substringp "だよ" text))
          (ros::ros-info "1")
          (let (spot-name)
            (setq spot-name (string-left-trim "ここが" (string-right-trim "だよ" text)))
            (ros::ros-info "spot-name: ~A" spot-name)
            (save-spot)
            (ros::ros-info "2")
            (send self :add-spot (save-spot) spot-name)
            ;; (send *ri* :speak-jp (format nil "~Aを覚えました" spot-name)
            (ros::ros-info (format nil "~Aを覚えました" spot-name))
            (when (string= spot-name goal-name)
              (setq prev-n nil)
              (setq mode :idling)
              (send *ri* :speak-jp (format nil "~Aに到着しました" goal-name)))))
         (t nil))))

  (:run ()
        (case mode
          (:idling t)
          
          (:record
           (let ((spot-coords (save-spot)))
             ;; if current pos is far enough from previous spot
             (when (> (abs (get-diff-position spot-coords (send prev-n :get :coords))) 1.0)
               (send self :add-spot spot-coords))))

          (:navigation
           (let* ((next (pop path)) (diff (convert-go-to-robot next)) (x (car diff)) (d (cadr diff))) 
             (send *ri* :go-pos-unsafe 0 0 d)
             (send *ri* :go-pos-unsafe x 0 0))
           (when (eq (length path) 0)
             (send *ri* :speak-jp (format nil "~Aに到着しました" goal))
             (setq mode :idling)
             (setq path nil))))))

(setq *ts* (instance teach-spot :init))
(ros::rate 2)
(do-until-key
 (ros::spin-once)
 (send *ts* :run)
 (ros::sleep))



