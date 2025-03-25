;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DoSensorFocusAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DoSensorFocusAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DOSENSORFOCUSACTION")
  (make-package "KORTEX_DRIVER::DOSENSORFOCUSACTION"))
(unless (find-package "KORTEX_DRIVER::DOSENSORFOCUSACTIONREQUEST")
  (make-package "KORTEX_DRIVER::DOSENSORFOCUSACTIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::DOSENSORFOCUSACTIONRESPONSE")
  (make-package "KORTEX_DRIVER::DOSENSORFOCUSACTIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DoSensorFocusActionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DoSensorFocusActionRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SensorFocusAction :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SensorFocusAction _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SensorFocusAction _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SensorFocusAction _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DoSensorFocusActionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DoSensorFocusActionResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DoSensorFocusAction
  :super ros::object
  :slots ())

(setf (get kortex_driver::DoSensorFocusAction :md5sum-) "a727a16574ce67bd4de8fba7da857f9f")
(setf (get kortex_driver::DoSensorFocusAction :datatype-) "kortex_driver/DoSensorFocusAction")
(setf (get kortex_driver::DoSensorFocusAction :request) kortex_driver::DoSensorFocusActionRequest)
(setf (get kortex_driver::DoSensorFocusAction :response) kortex_driver::DoSensorFocusActionResponse)

(defmethod kortex_driver::DoSensorFocusActionRequest
  (:response () (instance kortex_driver::DoSensorFocusActionResponse :init)))

(setf (get kortex_driver::DoSensorFocusActionRequest :md5sum-) "a727a16574ce67bd4de8fba7da857f9f")
(setf (get kortex_driver::DoSensorFocusActionRequest :datatype-) "kortex_driver/DoSensorFocusActionRequest")
(setf (get kortex_driver::DoSensorFocusActionRequest :definition-)
      "SensorFocusAction input

================================================================================
MSG: kortex_driver/SensorFocusAction

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

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DoSensorFocusActionResponse :md5sum-) "a727a16574ce67bd4de8fba7da857f9f")
(setf (get kortex_driver::DoSensorFocusActionResponse :datatype-) "kortex_driver/DoSensorFocusActionResponse")
(setf (get kortex_driver::DoSensorFocusActionResponse :definition-)
      "SensorFocusAction input

================================================================================
MSG: kortex_driver/SensorFocusAction

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

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DoSensorFocusAction "a727a16574ce67bd4de8fba7da857f9f")


