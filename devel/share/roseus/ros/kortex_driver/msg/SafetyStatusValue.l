;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyStatusValue)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyStatusValue (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE")
  (make-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))

(in-package "ROS")
;;//! \htmlinclude SafetyStatusValue.msg.html


(intern "*SAFETY_STATUS_UNSPECIFIED*" (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(shadow '*SAFETY_STATUS_UNSPECIFIED* (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(defconstant kortex_driver::SafetyStatusValue::*SAFETY_STATUS_UNSPECIFIED* 0)
(intern "*SAFETY_STATUS_WARNING*" (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(shadow '*SAFETY_STATUS_WARNING* (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(defconstant kortex_driver::SafetyStatusValue::*SAFETY_STATUS_WARNING* 1)
(intern "*SAFETY_STATUS_ERROR*" (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(shadow '*SAFETY_STATUS_ERROR* (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(defconstant kortex_driver::SafetyStatusValue::*SAFETY_STATUS_ERROR* 2)
(intern "*SAFETY_STATUS_NORMAL*" (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(shadow '*SAFETY_STATUS_NORMAL* (find-package "KORTEX_DRIVER::SAFETYSTATUSVALUE"))
(defconstant kortex_driver::SafetyStatusValue::*SAFETY_STATUS_NORMAL* 3)

(defun kortex_driver::SafetyStatusValue-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::SafetyStatusValue::*SAFETY_STATUS_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::SafetyStatusValue::*SAFETY_STATUS_WARNING*)
        ((= const 2) 'kortex_driver::SafetyStatusValue::*SAFETY_STATUS_ERROR*)
        ((= const 3) 'kortex_driver::SafetyStatusValue::*SAFETY_STATUS_NORMAL*)
        (t nil)))

(defclass kortex_driver::SafetyStatusValue
  :super ros::object
  :slots ())

(defmethod kortex_driver::SafetyStatusValue
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

(setf (get kortex_driver::SafetyStatusValue :md5sum-) "6f4f91637eeaf53ea8343202803cf904")
(setf (get kortex_driver::SafetyStatusValue :datatype-) "kortex_driver/SafetyStatusValue")
(setf (get kortex_driver::SafetyStatusValue :definition-)
      "
uint32 SAFETY_STATUS_UNSPECIFIED = 0

uint32 SAFETY_STATUS_WARNING = 1

uint32 SAFETY_STATUS_ERROR = 2

uint32 SAFETY_STATUS_NORMAL = 3

")



(provide :kortex_driver/SafetyStatusValue "6f4f91637eeaf53ea8343202803cf904")


