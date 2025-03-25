;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SensorFocusAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SensorFocusAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENSORFOCUSACTION")
  (make-package "KORTEX_DRIVER::SENSORFOCUSACTION"))

(in-package "ROS")
;;//! \htmlinclude SensorFocusAction.msg.html


(defclass kortex_driver::SensorFocusAction
  :super ros::object
  :slots (_sensor _focus_action _oneof_action_parameters ))

(defmethod kortex_driver::SensorFocusAction
  (:init
   (&key
    ((:sensor __sensor) 0)
    ((:focus_action __focus_action) 0)
    ((:oneof_action_parameters __oneof_action_parameters) (instance kortex_driver::SensorFocusAction_action_parameters :init))
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   (setq _focus_action (round __focus_action))
   (setq _oneof_action_parameters __oneof_action_parameters)
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:focus_action
   (&optional __focus_action)
   (if __focus_action (setq _focus_action __focus_action)) _focus_action)
  (:oneof_action_parameters
   (&rest __oneof_action_parameters)
   (if (keywordp (car __oneof_action_parameters))
       (send* _oneof_action_parameters __oneof_action_parameters)
     (progn
       (if __oneof_action_parameters (setq _oneof_action_parameters (car __oneof_action_parameters)))
       _oneof_action_parameters)))
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ;; uint32 _focus_action
    4
    ;; kortex_driver/SensorFocusAction_action_parameters _oneof_action_parameters
    (send _oneof_action_parameters :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sensor
       (write-long _sensor s)
     ;; uint32 _focus_action
       (write-long _focus_action s)
     ;; kortex_driver/SensorFocusAction_action_parameters _oneof_action_parameters
       (send _oneof_action_parameters :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _focus_action
     (setq _focus_action (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/SensorFocusAction_action_parameters _oneof_action_parameters
     (send _oneof_action_parameters :deserialize buf ptr-) (incf ptr- (send _oneof_action_parameters :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::SensorFocusAction :md5sum-) "cae3081e3d5f90fbdd9351620859003c")
(setf (get kortex_driver::SensorFocusAction :datatype-) "kortex_driver/SensorFocusAction")
(setf (get kortex_driver::SensorFocusAction :definition-)
      "
uint32 sensor
uint32 focus_action
SensorFocusAction_action_parameters oneof_action_parameters
================================================================================
MSG: kortex_driver/SensorFocusAction_action_parameters

FocusPoint[] focus_point
ManualFocus[] manual_focus
================================================================================
MSG: kortex_driver/FocusPoint

uint32 x
uint32 y
================================================================================
MSG: kortex_driver/ManualFocus

uint32 value
")



(provide :kortex_driver/SensorFocusAction "cae3081e3d5f90fbdd9351620859003c")


