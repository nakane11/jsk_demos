(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "speech_recognition_msgs")
(load "package://hand_navigation_pr2/euslisp/give-hand-pose.l")
(load "package://hand_navigation_pr2/euslisp/get-coords-diff.l")

(ros::roseus "teach_spot")
(pr2-init nil)

(defclass teach-spot
    :slots (mode spot-graph path prev-n goal-name))

(defmethod teach-spot
    (:init ()
     (setq mode :idling)
     (setq spot-graph (instance graph :init))
     (ros::subscribe "/Tablet/voice" speech_recognition_msgs::SpeechRecognitionCandidates #'send self :voice-cb))

  (:add-spot (spot &optional name)
     (let (n (instance node :init (or name (format nil "~A" (ros::time-now)))))
       (send n :put :coords spot)
       (send spot-graph :add-node n)
       (when prev-n
         (send spot-graph :add-arc-from-to prev-n n :both t))
       (setq prev-n n)))

  (:voice-cb (msg)
       (let ((text (elt (send msg :transcript) 0)))
         (cond
           ((substringp "に行こう" text)
            (setq goal-name (string-right-trim "に行こう" text))
            (send *ri* :nod)
            (let ((spot (save-spot)) start-n goal-n)
              (when () ;; if node exists nearby
                (setq start-n  ) ;; set start node from graph
                (when () ;; if goal in graph
                  (setq goal-n   ) ;; set goal node from graph
                  (when ();; if solver returns path from start-n to goal-n
                    (setq path )
                    (send *ri* :speak-jp "手をつなぎましょう" :wait t)
                    ;;give-hand-pose
                    ;;wait-for-grasp
                    (send *ri* :speak-jp "出発します")
                    (setq mode :navigation)
                    (return-from :voice-cb)))
                (setq prev-n start-n))) ;; link existing node and starting point
            ;; else entering record mode
            (send *ri* :speak-jp "どこにあるのですか" :wait t)
            (send *ri* :speak-jp "連れて行ってください"))
         
         ;; ((substringp "手をつなごう" text)
         ;;  ;;give-hand-pose
         ;;  )
         
         ((substringp "道順を覚えて" text)
          (send *ri* :speak-jp "はい")
          (send *ri* :nod)          
          (send self :add-spot (save-spot))
          (setq mode :record))

         ((and (substringp "ここが" text) (substringp "だよ" text))
          (let (spot-name (string-left-trim "ここが" (string-right-trim "だよ" text)))
            (send self :add-spot (save-spot) spot-name)
            (when (string= spot-name goal-name)
              (setq prev-n nil)
              (setq mode :idling)
              (send *ri* :speak-jp (format nil "~Aを覚えました" goal-name))))))))

  (:run ()
        (case mode
          (:idling t)
          
          (:record
           (let ((spot (save-sot)))
             ;; if current pos is far enough from previous spot
             (when (> (get-diff-position spot (send prev-n :get :coords)) 1.0)
               (send self :add-spot spot))))

          (:navigation
           (let* ((next (pop path)) (diff (convert-go-to-robot next)) (x (car diff)) (d (cadr diff))) 
             (send *ri* :go-pos-unsafe 0 0 d)
             (send *ri* :go-pos-unsafe x 0 0))
           (when (eq (length path) 0)
             (send *ri* :speak-jp (format nil "~Aに到着しました" goal))
             (setq mode :idling)
             (setq path nil))))))

(setq *ts* (instance teach-spot :init))
(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (send *ts* :run)
 (ros::sleep))



