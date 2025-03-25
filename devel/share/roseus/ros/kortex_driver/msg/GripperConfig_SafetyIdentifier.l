;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GripperConfig_SafetyIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GripperConfig_SafetyIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER")
  (make-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude GripperConfig_SafetyIdentifier.msg.html


(intern "*UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER* 0)
(intern "*MAXIMUM_MOTOR_CURRENT*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_MOTOR_CURRENT* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_MOTOR_CURRENT* 1)
(intern "*MAXIMUM_VOLTAGE*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_VOLTAGE* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_VOLTAGE* 2)
(intern "*MINIMUM_VOLTAGE*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*MINIMUM_VOLTAGE* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*MINIMUM_VOLTAGE* 4)
(intern "*MAXIMUM_MOTOR_TEMPERATURE*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_MOTOR_TEMPERATURE* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_MOTOR_TEMPERATURE* 8)
(intern "*MAXIMUM_CORE_TEMPERATURE*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_CORE_TEMPERATURE* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_CORE_TEMPERATURE* 16)
(intern "*NON_VOLATILE_MEMORY_CORRUPTED*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*NON_VOLATILE_MEMORY_CORRUPTED* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*NON_VOLATILE_MEMORY_CORRUPTED* 32)
(intern "*EMERGENCY_LINE_ASSERTED*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*EMERGENCY_LINE_ASSERTED* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*EMERGENCY_LINE_ASSERTED* 64)
(intern "*COMMUNICATION_TICK_LOST*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*COMMUNICATION_TICK_LOST* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*COMMUNICATION_TICK_LOST* 128)
(intern "*WATCHDOG_TRIGGERED*" (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(shadow '*WATCHDOG_TRIGGERED* (find-package "KORTEX_DRIVER::GRIPPERCONFIG_SAFETYIDENTIFIER"))
(defconstant kortex_driver::GripperConfig_SafetyIdentifier::*WATCHDOG_TRIGGERED* 256)

(defun kortex_driver::GripperConfig_SafetyIdentifier-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GripperConfig_SafetyIdentifier::*UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER*)
        ((= const 1) 'kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_MOTOR_CURRENT*)
        ((= const 2) 'kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_VOLTAGE*)
        ((= const 4) 'kortex_driver::GripperConfig_SafetyIdentifier::*MINIMUM_VOLTAGE*)
        ((= const 8) 'kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_MOTOR_TEMPERATURE*)
        ((= const 16) 'kortex_driver::GripperConfig_SafetyIdentifier::*MAXIMUM_CORE_TEMPERATURE*)
        ((= const 32) 'kortex_driver::GripperConfig_SafetyIdentifier::*NON_VOLATILE_MEMORY_CORRUPTED*)
        ((= const 64) 'kortex_driver::GripperConfig_SafetyIdentifier::*EMERGENCY_LINE_ASSERTED*)
        ((= const 128) 'kortex_driver::GripperConfig_SafetyIdentifier::*COMMUNICATION_TICK_LOST*)
        ((= const 256) 'kortex_driver::GripperConfig_SafetyIdentifier::*WATCHDOG_TRIGGERED*)
        (t nil)))

(defclass kortex_driver::GripperConfig_SafetyIdentifier
  :super ros::object
  :slots ())

(defmethod kortex_driver::GripperConfig_SafetyIdentifier
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::GripperConfig_SafetyIdentifier :md5sum-) "d7392d34e7992e1d68eccf5c1f5cd7f7")
(setf (get kortex_driver::GripperConfig_SafetyIdentifier :datatype-) "kortex_driver/GripperConfig_SafetyIdentifier")
(setf (get kortex_driver::GripperConfig_SafetyIdentifier :definition-)
      "
uint32 UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER = 0

uint32 MAXIMUM_MOTOR_CURRENT = 1

uint32 MAXIMUM_VOLTAGE = 2

uint32 MINIMUM_VOLTAGE = 4

uint32 MAXIMUM_MOTOR_TEMPERATURE = 8

uint32 MAXIMUM_CORE_TEMPERATURE = 16

uint32 NON_VOLATILE_MEMORY_CORRUPTED = 32

uint32 EMERGENCY_LINE_ASSERTED = 64

uint32 COMMUNICATION_TICK_LOST = 128

uint32 WATCHDOG_TRIGGERED = 256

")



(provide :kortex_driver/GripperConfig_SafetyIdentifier "d7392d34e7992e1d68eccf5c1f5cd7f7")


