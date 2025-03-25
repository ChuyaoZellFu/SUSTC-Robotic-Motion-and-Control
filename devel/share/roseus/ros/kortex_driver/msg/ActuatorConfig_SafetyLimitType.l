;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorConfig_SafetyLimitType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorConfig_SafetyLimitType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SAFETYLIMITTYPE")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_SAFETYLIMITTYPE"))

(in-package "ROS")
;;//! \htmlinclude ActuatorConfig_SafetyLimitType.msg.html


(intern "*MAXIMAL_LIMIT*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SAFETYLIMITTYPE"))
(shadow '*MAXIMAL_LIMIT* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SAFETYLIMITTYPE"))
(defconstant kortex_driver::ActuatorConfig_SafetyLimitType::*MAXIMAL_LIMIT* 0)
(intern "*MINIMAL_LIMIT*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SAFETYLIMITTYPE"))
(shadow '*MINIMAL_LIMIT* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SAFETYLIMITTYPE"))
(defconstant kortex_driver::ActuatorConfig_SafetyLimitType::*MINIMAL_LIMIT* 1)

(defun kortex_driver::ActuatorConfig_SafetyLimitType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ActuatorConfig_SafetyLimitType::*MAXIMAL_LIMIT*)
        ((= const 1) 'kortex_driver::ActuatorConfig_SafetyLimitType::*MINIMAL_LIMIT*)
        (t nil)))

(defclass kortex_driver::ActuatorConfig_SafetyLimitType
  :super ros::object
  :slots ())

(defmethod kortex_driver::ActuatorConfig_SafetyLimitType
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

(setf (get kortex_driver::ActuatorConfig_SafetyLimitType :md5sum-) "2886a0a967849f2573bf0fd798659a83")
(setf (get kortex_driver::ActuatorConfig_SafetyLimitType :datatype-) "kortex_driver/ActuatorConfig_SafetyLimitType")
(setf (get kortex_driver::ActuatorConfig_SafetyLimitType :definition-)
      "
uint32 MAXIMAL_LIMIT = 0

uint32 MINIMAL_LIMIT = 1

")



(provide :kortex_driver/ActuatorConfig_SafetyLimitType "2886a0a967849f2573bf0fd798659a83")


