;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_SafetyIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_SafetyIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER")
  (make-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude Base_SafetyIdentifier.msg.html


(intern "*UNSPECIFIED_BASE_SAFETY_IDENTIFIER*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*UNSPECIFIED_BASE_SAFETY_IDENTIFIER* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*UNSPECIFIED_BASE_SAFETY_IDENTIFIER* 0)
(intern "*FIRMWARE_UPDATE_FAILURE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*FIRMWARE_UPDATE_FAILURE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*FIRMWARE_UPDATE_FAILURE* 1)
(intern "*EXTERNAL_COMMUNICATION_ERROR*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*EXTERNAL_COMMUNICATION_ERROR* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*EXTERNAL_COMMUNICATION_ERROR* 2)
(intern "*MAXIMUM_AMBIENT_TEMPERATURE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_AMBIENT_TEMPERATURE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_AMBIENT_TEMPERATURE* 4)
(intern "*MAXIMUM_CORE_TEMPERATURE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_CORE_TEMPERATURE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_CORE_TEMPERATURE* 8)
(intern "*JOINT_FAULT*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*JOINT_FAULT* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*JOINT_FAULT* 16)
(intern "*CYCLIC_DATA_JITTER*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*CYCLIC_DATA_JITTER* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*CYCLIC_DATA_JITTER* 32)
(intern "*REACHED_MAXIMUM_EVENT_LOGS*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*REACHED_MAXIMUM_EVENT_LOGS* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*REACHED_MAXIMUM_EVENT_LOGS* 64)
(intern "*NO_KINEMATICS_SUPPORT*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*NO_KINEMATICS_SUPPORT* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*NO_KINEMATICS_SUPPORT* 128)
(intern "*BRAKE_REMOVAL_FAILURE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*BRAKE_REMOVAL_FAILURE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*BRAKE_REMOVAL_FAILURE* 256)
(intern "*NETWORK_ERROR*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*NETWORK_ERROR* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*NETWORK_ERROR* 512)
(intern "*UNABLE_TO_REACH_POSE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*UNABLE_TO_REACH_POSE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*UNABLE_TO_REACH_POSE* 1024)
(intern "*JOINT_DETECTION_ERROR*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*JOINT_DETECTION_ERROR* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*JOINT_DETECTION_ERROR* 2048)
(intern "*NETWORK_INITIALIZATION_ERROR*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*NETWORK_INITIALIZATION_ERROR* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*NETWORK_INITIALIZATION_ERROR* 4096)
(intern "*MAXIMUM_CURRENT*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_CURRENT* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_CURRENT* 8192)
(intern "*MAXIMUM_VOLTAGE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_VOLTAGE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_VOLTAGE* 16384)
(intern "*MINIMUM_VOLTAGE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MINIMUM_VOLTAGE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MINIMUM_VOLTAGE* 32768)
(intern "*MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY* 65536)
(intern "*MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY* 131072)
(intern "*MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION* 262144)
(intern "*MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION* 524288)
(intern "*MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE* 1048576)
(intern "*MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE* 2097152)
(intern "*MAXIMUM_END_EFFECTOR_PAYLOAD*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*MAXIMUM_END_EFFECTOR_PAYLOAD* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_PAYLOAD* 4194304)
(intern "*EMERGENCY_STOP_ACTIVATED*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*EMERGENCY_STOP_ACTIVATED* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*EMERGENCY_STOP_ACTIVATED* 8388608)
(intern "*EMERGENCY_LINE_ACTIVATED*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*EMERGENCY_LINE_ACTIVATED* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*EMERGENCY_LINE_ACTIVATED* 16777216)
(intern "*INRUSH_CURRENT_LIMITER_FAULT*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*INRUSH_CURRENT_LIMITER_FAULT* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*INRUSH_CURRENT_LIMITER_FAULT* 33554432)
(intern "*NVRAM_CORRUPTED*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*NVRAM_CORRUPTED* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*NVRAM_CORRUPTED* 67108864)
(intern "*INCOMPATIBLE_FIRMWARE_VERSION*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*INCOMPATIBLE_FIRMWARE_VERSION* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*INCOMPATIBLE_FIRMWARE_VERSION* 134217728)
(intern "*POWERON_SELF_TEST_FAILURE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*POWERON_SELF_TEST_FAILURE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*POWERON_SELF_TEST_FAILURE* 268435456)
(intern "*DISCRETE_INPUT_STUCK_ACTIVE*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*DISCRETE_INPUT_STUCK_ACTIVE* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*DISCRETE_INPUT_STUCK_ACTIVE* 536870912)
(intern "*ARM_INTO_ILLEGAL_POSITION*" (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(shadow '*ARM_INTO_ILLEGAL_POSITION* (find-package "KORTEX_DRIVER::BASE_SAFETYIDENTIFIER"))
(defconstant kortex_driver::Base_SafetyIdentifier::*ARM_INTO_ILLEGAL_POSITION* 1073741824)

(defun kortex_driver::Base_SafetyIdentifier-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Base_SafetyIdentifier::*UNSPECIFIED_BASE_SAFETY_IDENTIFIER*)
        ((= const 1) 'kortex_driver::Base_SafetyIdentifier::*FIRMWARE_UPDATE_FAILURE*)
        ((= const 2) 'kortex_driver::Base_SafetyIdentifier::*EXTERNAL_COMMUNICATION_ERROR*)
        ((= const 4) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_AMBIENT_TEMPERATURE*)
        ((= const 8) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_CORE_TEMPERATURE*)
        ((= const 16) 'kortex_driver::Base_SafetyIdentifier::*JOINT_FAULT*)
        ((= const 32) 'kortex_driver::Base_SafetyIdentifier::*CYCLIC_DATA_JITTER*)
        ((= const 64) 'kortex_driver::Base_SafetyIdentifier::*REACHED_MAXIMUM_EVENT_LOGS*)
        ((= const 128) 'kortex_driver::Base_SafetyIdentifier::*NO_KINEMATICS_SUPPORT*)
        ((= const 256) 'kortex_driver::Base_SafetyIdentifier::*BRAKE_REMOVAL_FAILURE*)
        ((= const 512) 'kortex_driver::Base_SafetyIdentifier::*NETWORK_ERROR*)
        ((= const 1024) 'kortex_driver::Base_SafetyIdentifier::*UNABLE_TO_REACH_POSE*)
        ((= const 2048) 'kortex_driver::Base_SafetyIdentifier::*JOINT_DETECTION_ERROR*)
        ((= const 4096) 'kortex_driver::Base_SafetyIdentifier::*NETWORK_INITIALIZATION_ERROR*)
        ((= const 8192) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_CURRENT*)
        ((= const 16384) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_VOLTAGE*)
        ((= const 32768) 'kortex_driver::Base_SafetyIdentifier::*MINIMUM_VOLTAGE*)
        ((= const 65536) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY*)
        ((= const 131072) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY*)
        ((= const 262144) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION*)
        ((= const 524288) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION*)
        ((= const 1048576) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE*)
        ((= const 2097152) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE*)
        ((= const 4194304) 'kortex_driver::Base_SafetyIdentifier::*MAXIMUM_END_EFFECTOR_PAYLOAD*)
        ((= const 8388608) 'kortex_driver::Base_SafetyIdentifier::*EMERGENCY_STOP_ACTIVATED*)
        ((= const 16777216) 'kortex_driver::Base_SafetyIdentifier::*EMERGENCY_LINE_ACTIVATED*)
        ((= const 33554432) 'kortex_driver::Base_SafetyIdentifier::*INRUSH_CURRENT_LIMITER_FAULT*)
        ((= const 67108864) 'kortex_driver::Base_SafetyIdentifier::*NVRAM_CORRUPTED*)
        ((= const 134217728) 'kortex_driver::Base_SafetyIdentifier::*INCOMPATIBLE_FIRMWARE_VERSION*)
        ((= const 268435456) 'kortex_driver::Base_SafetyIdentifier::*POWERON_SELF_TEST_FAILURE*)
        ((= const 536870912) 'kortex_driver::Base_SafetyIdentifier::*DISCRETE_INPUT_STUCK_ACTIVE*)
        ((= const 1073741824) 'kortex_driver::Base_SafetyIdentifier::*ARM_INTO_ILLEGAL_POSITION*)
        (t nil)))

(defclass kortex_driver::Base_SafetyIdentifier
  :super ros::object
  :slots ())

(defmethod kortex_driver::Base_SafetyIdentifier
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

(setf (get kortex_driver::Base_SafetyIdentifier :md5sum-) "1af24619c0f6e3d61a2eb98a96e27e96")
(setf (get kortex_driver::Base_SafetyIdentifier :datatype-) "kortex_driver/Base_SafetyIdentifier")
(setf (get kortex_driver::Base_SafetyIdentifier :definition-)
      "
uint32 UNSPECIFIED_BASE_SAFETY_IDENTIFIER = 0

uint32 FIRMWARE_UPDATE_FAILURE = 1

uint32 EXTERNAL_COMMUNICATION_ERROR = 2

uint32 MAXIMUM_AMBIENT_TEMPERATURE = 4

uint32 MAXIMUM_CORE_TEMPERATURE = 8

uint32 JOINT_FAULT = 16

uint32 CYCLIC_DATA_JITTER = 32

uint32 REACHED_MAXIMUM_EVENT_LOGS = 64

uint32 NO_KINEMATICS_SUPPORT = 128

uint32 BRAKE_REMOVAL_FAILURE = 256

uint32 NETWORK_ERROR = 512

uint32 UNABLE_TO_REACH_POSE = 1024

uint32 JOINT_DETECTION_ERROR = 2048

uint32 NETWORK_INITIALIZATION_ERROR = 4096

uint32 MAXIMUM_CURRENT = 8192

uint32 MAXIMUM_VOLTAGE = 16384

uint32 MINIMUM_VOLTAGE = 32768

uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY = 65536

uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY = 131072

uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION = 262144

uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION = 524288

uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE = 1048576

uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE = 2097152

uint32 MAXIMUM_END_EFFECTOR_PAYLOAD = 4194304

uint32 EMERGENCY_STOP_ACTIVATED = 8388608

uint32 EMERGENCY_LINE_ACTIVATED = 16777216

uint32 INRUSH_CURRENT_LIMITER_FAULT = 33554432

uint32 NVRAM_CORRUPTED = 67108864

uint32 INCOMPATIBLE_FIRMWARE_VERSION = 134217728

uint32 POWERON_SELF_TEST_FAILURE = 268435456

uint32 DISCRETE_INPUT_STUCK_ACTIVE = 536870912

uint32 ARM_INTO_ILLEGAL_POSITION = 1073741824

")



(provide :kortex_driver/Base_SafetyIdentifier "1af24619c0f6e3d61a2eb98a96e27e96")


