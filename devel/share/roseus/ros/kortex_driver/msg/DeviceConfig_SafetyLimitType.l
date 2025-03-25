;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeviceConfig_SafetyLimitType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeviceConfig_SafetyLimitType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE")
  (make-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))

(in-package "ROS")
;;//! \htmlinclude DeviceConfig_SafetyLimitType.msg.html


(intern "*UNSPECIFIED_SAFETY_LIMIT_TYPE*" (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(shadow '*UNSPECIFIED_SAFETY_LIMIT_TYPE* (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(defconstant kortex_driver::DeviceConfig_SafetyLimitType::*UNSPECIFIED_SAFETY_LIMIT_TYPE* 0)
(intern "*MINIMAL_LIMIT*" (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(shadow '*MINIMAL_LIMIT* (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(defconstant kortex_driver::DeviceConfig_SafetyLimitType::*MINIMAL_LIMIT* 1)
(intern "*MAXIMAL_LIMIT*" (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(shadow '*MAXIMAL_LIMIT* (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(defconstant kortex_driver::DeviceConfig_SafetyLimitType::*MAXIMAL_LIMIT* 2)
(intern "*EVENT_LIMIT*" (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(shadow '*EVENT_LIMIT* (find-package "KORTEX_DRIVER::DEVICECONFIG_SAFETYLIMITTYPE"))
(defconstant kortex_driver::DeviceConfig_SafetyLimitType::*EVENT_LIMIT* 3)

(defun kortex_driver::DeviceConfig_SafetyLimitType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::DeviceConfig_SafetyLimitType::*UNSPECIFIED_SAFETY_LIMIT_TYPE*)
        ((= const 1) 'kortex_driver::DeviceConfig_SafetyLimitType::*MINIMAL_LIMIT*)
        ((= const 2) 'kortex_driver::DeviceConfig_SafetyLimitType::*MAXIMAL_LIMIT*)
        ((= const 3) 'kortex_driver::DeviceConfig_SafetyLimitType::*EVENT_LIMIT*)
        (t nil)))

(defclass kortex_driver::DeviceConfig_SafetyLimitType
  :super ros::object
  :slots ())

(defmethod kortex_driver::DeviceConfig_SafetyLimitType
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

(setf (get kortex_driver::DeviceConfig_SafetyLimitType :md5sum-) "497794fa9cad838eb83949e4e871d8e4")
(setf (get kortex_driver::DeviceConfig_SafetyLimitType :datatype-) "kortex_driver/DeviceConfig_SafetyLimitType")
(setf (get kortex_driver::DeviceConfig_SafetyLimitType :definition-)
      "
uint32 UNSPECIFIED_SAFETY_LIMIT_TYPE = 0

uint32 MINIMAL_LIMIT = 1

uint32 MAXIMAL_LIMIT = 2

uint32 EVENT_LIMIT = 3

")



(provide :kortex_driver/DeviceConfig_SafetyLimitType "497794fa9cad838eb83949e4e871d8e4")


