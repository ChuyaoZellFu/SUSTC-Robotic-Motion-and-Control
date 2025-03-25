;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlConfigurationEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlConfigurationEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT")
  (make-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))

(in-package "ROS")
;;//! \htmlinclude ControlConfigurationEvent.msg.html


(intern "*UNSPECIFIED_CONTROL_CONFIGURATION_EVENT*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*UNSPECIFIED_CONTROL_CONFIGURATION_EVENT* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*UNSPECIFIED_CONTROL_CONFIGURATION_EVENT* 0)
(intern "*ANGLE_UNIT_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*ANGLE_UNIT_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*ANGLE_UNIT_CHANGED* 1)
(intern "*GRAVITY_VECTOR_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*GRAVITY_VECTOR_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*GRAVITY_VECTOR_CHANGED* 2)
(intern "*JOINT_ADMITTANCE_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*JOINT_ADMITTANCE_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*JOINT_ADMITTANCE_CONFIGURATION_CHANGED* 4)
(intern "*NULL_ADMITTANCE_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*NULL_ADMITTANCE_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*NULL_ADMITTANCE_CONFIGURATION_CHANGED* 5)
(intern "*CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED* 6)
(intern "*JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED* 7)
(intern "*WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED* 8)
(intern "*WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED* 9)
(intern "*CONTROL_CONFIGURATION_FACTORY_RESTORED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*CONTROL_CONFIGURATION_FACTORY_RESTORED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*CONTROL_CONFIGURATION_FACTORY_RESTORED* 10)
(intern "*TOOL_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*TOOL_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*TOOL_CONFIGURATION_CHANGED* 11)
(intern "*PAYLOAD_CONFIGURATION_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*PAYLOAD_CONFIGURATION_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*PAYLOAD_CONFIGURATION_CHANGED* 12)
(intern "*CARTESIAN_REFERENCE_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*CARTESIAN_REFERENCE_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*CARTESIAN_REFERENCE_CHANGED* 13)
(intern "*CHANGE_CONTROL_MODE_FAILED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*CHANGE_CONTROL_MODE_FAILED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*CHANGE_CONTROL_MODE_FAILED* 14)
(intern "*JOINT_SPEED_SOFT_LIMITS_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*JOINT_SPEED_SOFT_LIMITS_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*JOINT_SPEED_SOFT_LIMITS_CHANGED* 16)
(intern "*TWIST_LINEAR_SOFT_LIMIT_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*TWIST_LINEAR_SOFT_LIMIT_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*TWIST_LINEAR_SOFT_LIMIT_CHANGED* 17)
(intern "*TWIST_ANGULAR_SOFT_LIMIT_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*TWIST_ANGULAR_SOFT_LIMIT_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*TWIST_ANGULAR_SOFT_LIMIT_CHANGED* 18)
(intern "*JOINT_ACCELERATION_SOFT_LIMITS_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*JOINT_ACCELERATION_SOFT_LIMITS_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*JOINT_ACCELERATION_SOFT_LIMITS_CHANGED* 19)
(intern "*DESIRED_TWIST_LINEAR_SPEED_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*DESIRED_TWIST_LINEAR_SPEED_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*DESIRED_TWIST_LINEAR_SPEED_CHANGED* 20)
(intern "*DESIRED_TWIST_ANGULAR_SPEED_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*DESIRED_TWIST_ANGULAR_SPEED_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*DESIRED_TWIST_ANGULAR_SPEED_CHANGED* 21)
(intern "*DESIRED_JOINT_SPEED_CHANGED*" (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(shadow '*DESIRED_JOINT_SPEED_CHANGED* (find-package "KORTEX_DRIVER::CONTROLCONFIGURATIONEVENT"))
(defconstant kortex_driver::ControlConfigurationEvent::*DESIRED_JOINT_SPEED_CHANGED* 22)

(defun kortex_driver::ControlConfigurationEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ControlConfigurationEvent::*UNSPECIFIED_CONTROL_CONFIGURATION_EVENT*)
        ((= const 1) 'kortex_driver::ControlConfigurationEvent::*ANGLE_UNIT_CHANGED*)
        ((= const 2) 'kortex_driver::ControlConfigurationEvent::*GRAVITY_VECTOR_CHANGED*)
        ((= const 4) 'kortex_driver::ControlConfigurationEvent::*JOINT_ADMITTANCE_CONFIGURATION_CHANGED*)
        ((= const 5) 'kortex_driver::ControlConfigurationEvent::*NULL_ADMITTANCE_CONFIGURATION_CHANGED*)
        ((= const 6) 'kortex_driver::ControlConfigurationEvent::*CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED*)
        ((= const 7) 'kortex_driver::ControlConfigurationEvent::*JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED*)
        ((= const 8) 'kortex_driver::ControlConfigurationEvent::*WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED*)
        ((= const 9) 'kortex_driver::ControlConfigurationEvent::*WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED*)
        ((= const 10) 'kortex_driver::ControlConfigurationEvent::*CONTROL_CONFIGURATION_FACTORY_RESTORED*)
        ((= const 11) 'kortex_driver::ControlConfigurationEvent::*TOOL_CONFIGURATION_CHANGED*)
        ((= const 12) 'kortex_driver::ControlConfigurationEvent::*PAYLOAD_CONFIGURATION_CHANGED*)
        ((= const 13) 'kortex_driver::ControlConfigurationEvent::*CARTESIAN_REFERENCE_CHANGED*)
        ((= const 14) 'kortex_driver::ControlConfigurationEvent::*CHANGE_CONTROL_MODE_FAILED*)
        ((= const 16) 'kortex_driver::ControlConfigurationEvent::*JOINT_SPEED_SOFT_LIMITS_CHANGED*)
        ((= const 17) 'kortex_driver::ControlConfigurationEvent::*TWIST_LINEAR_SOFT_LIMIT_CHANGED*)
        ((= const 18) 'kortex_driver::ControlConfigurationEvent::*TWIST_ANGULAR_SOFT_LIMIT_CHANGED*)
        ((= const 19) 'kortex_driver::ControlConfigurationEvent::*JOINT_ACCELERATION_SOFT_LIMITS_CHANGED*)
        ((= const 20) 'kortex_driver::ControlConfigurationEvent::*DESIRED_TWIST_LINEAR_SPEED_CHANGED*)
        ((= const 21) 'kortex_driver::ControlConfigurationEvent::*DESIRED_TWIST_ANGULAR_SPEED_CHANGED*)
        ((= const 22) 'kortex_driver::ControlConfigurationEvent::*DESIRED_JOINT_SPEED_CHANGED*)
        (t nil)))

(defclass kortex_driver::ControlConfigurationEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::ControlConfigurationEvent
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

(setf (get kortex_driver::ControlConfigurationEvent :md5sum-) "5aa2a442ccb0c93e20aaef58c6149541")
(setf (get kortex_driver::ControlConfigurationEvent :datatype-) "kortex_driver/ControlConfigurationEvent")
(setf (get kortex_driver::ControlConfigurationEvent :definition-)
      "
uint32 UNSPECIFIED_CONTROL_CONFIGURATION_EVENT = 0

uint32 ANGLE_UNIT_CHANGED = 1

uint32 GRAVITY_VECTOR_CHANGED = 2

uint32 JOINT_ADMITTANCE_CONFIGURATION_CHANGED = 4

uint32 NULL_ADMITTANCE_CONFIGURATION_CHANGED = 5

uint32 CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED = 6

uint32 JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED = 7

uint32 WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED = 8

uint32 WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED = 9

uint32 CONTROL_CONFIGURATION_FACTORY_RESTORED = 10

uint32 TOOL_CONFIGURATION_CHANGED = 11

uint32 PAYLOAD_CONFIGURATION_CHANGED = 12

uint32 CARTESIAN_REFERENCE_CHANGED = 13

uint32 CHANGE_CONTROL_MODE_FAILED = 14

uint32 JOINT_SPEED_SOFT_LIMITS_CHANGED = 16

uint32 TWIST_LINEAR_SOFT_LIMIT_CHANGED = 17

uint32 TWIST_ANGULAR_SOFT_LIMIT_CHANGED = 18

uint32 JOINT_ACCELERATION_SOFT_LIMITS_CHANGED = 19

uint32 DESIRED_TWIST_LINEAR_SPEED_CHANGED = 20

uint32 DESIRED_TWIST_ANGULAR_SPEED_CHANGED = 21

uint32 DESIRED_JOINT_SPEED_CHANGED = 22

")



(provide :kortex_driver/ControlConfigurationEvent "5aa2a442ccb0c93e20aaef58c6149541")


