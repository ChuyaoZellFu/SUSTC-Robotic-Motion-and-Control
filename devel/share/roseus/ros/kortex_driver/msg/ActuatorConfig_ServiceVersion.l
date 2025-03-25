;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorConfig_ServiceVersion)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorConfig_ServiceVersion (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SERVICEVERSION")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_SERVICEVERSION"))

(in-package "ROS")
;;//! \htmlinclude ActuatorConfig_ServiceVersion.msg.html


(intern "*RESERVED_0*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SERVICEVERSION"))
(shadow '*RESERVED_0* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SERVICEVERSION"))
(defconstant kortex_driver::ActuatorConfig_ServiceVersion::*RESERVED_0* 0)
(intern "*CURRENT_VERSION*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SERVICEVERSION"))
(shadow '*CURRENT_VERSION* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_SERVICEVERSION"))
(defconstant kortex_driver::ActuatorConfig_ServiceVersion::*CURRENT_VERSION* 1)

(defun kortex_driver::ActuatorConfig_ServiceVersion-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ActuatorConfig_ServiceVersion::*RESERVED_0*)
        ((= const 1) 'kortex_driver::ActuatorConfig_ServiceVersion::*CURRENT_VERSION*)
        (t nil)))

(defclass kortex_driver::ActuatorConfig_ServiceVersion
  :super ros::object
  :slots ())

(defmethod kortex_driver::ActuatorConfig_ServiceVersion
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

(setf (get kortex_driver::ActuatorConfig_ServiceVersion :md5sum-) "d8624be8e7830df48d9964b7612fa5b3")
(setf (get kortex_driver::ActuatorConfig_ServiceVersion :datatype-) "kortex_driver/ActuatorConfig_ServiceVersion")
(setf (get kortex_driver::ActuatorConfig_ServiceVersion :definition-)
      "
uint32 RESERVED_0 = 0

uint32 CURRENT_VERSION = 1

")



(provide :kortex_driver/ActuatorConfig_ServiceVersion "d8624be8e7830df48d9964b7612fa5b3")


