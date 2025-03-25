;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorConfig_GetControlMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorConfig_GetControlMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_GETCONTROLMODE")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_GETCONTROLMODE"))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_GETCONTROLMODEREQUEST")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_GETCONTROLMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_GETCONTROLMODERESPONSE")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_GETCONTROLMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ActuatorConfig_GetControlModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ActuatorConfig_GetControlModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ActuatorConfig_GetControlModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ActuatorConfig_GetControlModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ActuatorConfig_ControlModeInformation :init))
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
    ;; kortex_driver/ActuatorConfig_ControlModeInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ActuatorConfig_ControlModeInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ActuatorConfig_ControlModeInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ActuatorConfig_GetControlMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::ActuatorConfig_GetControlMode :md5sum-) "da0700ae55dc292dd80209569315d03f")
(setf (get kortex_driver::ActuatorConfig_GetControlMode :datatype-) "kortex_driver/ActuatorConfig_GetControlMode")
(setf (get kortex_driver::ActuatorConfig_GetControlMode :request) kortex_driver::ActuatorConfig_GetControlModeRequest)
(setf (get kortex_driver::ActuatorConfig_GetControlMode :response) kortex_driver::ActuatorConfig_GetControlModeResponse)

(defmethod kortex_driver::ActuatorConfig_GetControlModeRequest
  (:response () (instance kortex_driver::ActuatorConfig_GetControlModeResponse :init)))

(setf (get kortex_driver::ActuatorConfig_GetControlModeRequest :md5sum-) "da0700ae55dc292dd80209569315d03f")
(setf (get kortex_driver::ActuatorConfig_GetControlModeRequest :datatype-) "kortex_driver/ActuatorConfig_GetControlModeRequest")
(setf (get kortex_driver::ActuatorConfig_GetControlModeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ActuatorConfig_ControlModeInformation output

================================================================================
MSG: kortex_driver/ActuatorConfig_ControlModeInformation

")

(setf (get kortex_driver::ActuatorConfig_GetControlModeResponse :md5sum-) "da0700ae55dc292dd80209569315d03f")
(setf (get kortex_driver::ActuatorConfig_GetControlModeResponse :datatype-) "kortex_driver/ActuatorConfig_GetControlModeResponse")
(setf (get kortex_driver::ActuatorConfig_GetControlModeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ActuatorConfig_ControlModeInformation output

================================================================================
MSG: kortex_driver/ActuatorConfig_ControlModeInformation

")



(provide :kortex_driver/ActuatorConfig_GetControlMode "da0700ae55dc292dd80209569315d03f")


