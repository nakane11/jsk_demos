#!/usr/bin/env roseus

(load "package://welcome_to_jsk_fetch/euslisp/utils.l")

(defun main ()
  ;; Init fetch-interface
  (fetch-init)
  (setq vf-client (instance virtual-force-client :init "/end_effector/wrench_transformed"))

  ;; Reset-pose
  ;; (send *fetch* :reset-pose)
  (send *ri* :raise-arm-pose)
  (send *fetch* :torso_lift_joint :joint-angle 500)
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; Register place names to julius
  (register-spots-to-julius)

  (do-until-key
    (wait-for-impact vf-client)
    ;; Wait for person by face recognition and then greeting
    ;; (send *ri* :speak-jp "案内する人を探します" :wait t)
    (send *ri* :speak-jp "はい" :wait t)
    (greeting :use-people-detection nil :use-face-detection nil)
    (salute-pose)

    ;; Ask destination
    (send *ri* :speak-jp "どこにいきますか？" :wait t)
    (unix:sleep 2) ;; Wait 2 seconds not to recognize fetch's own voice at next function

    ;; Go to spot from speech
    (send *ri* :raise-arm-pose)
    (send *ri* :wait-interpolation)
    (go-to-spot-from-speech :vf-client vf-client)))
