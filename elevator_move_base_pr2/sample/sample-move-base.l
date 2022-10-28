#!/usr/bin/env roseus
;; -*- coding: utf-8 -*-                                                                                                                                                                                                                                                           

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :app-utils "package://jsk_demo_common/euslisp/app-utils.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :eng2-scene "package://jsk_maps/src/eng8-scene.l")

(load "package://elevator_move_base_pr2/src/utils.l")


(defvar *initial-pose*)
(defvar *pr2*)
(defvar *ri*)
(defvar *scene*)

(defun init ()
  (setq *pr2* (pr2))
  (setq *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base"))
  ;; (send *pr2* :reset-pose)                                                                                                                                                                                                                                                      
  ;; (pr2-tuckarm-pose)                                                                                                                                                                                                                                                            
  ;; (look-front)                                                                                                                                                                                                                                                                  
  (setq *scene* (make-eng8-scene))
  (setq *initial-pose* (send *ri* :state :worldcoords)))

;; (send *ri* :move-to (send *scene* :spot "/eng8/6f/room610-chair-back"))                                                                                                                                                                                                         

