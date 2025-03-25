;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OperatingMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OperatingMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPERATINGMODE")
  (make-package "KORTEX_DRIVER::OPERATINGMODE"))

(in-package "ROS")
;;//! \htmlinclude OperatingMode.msg.html


(intern "*UNSPECIFIED_OPERATING_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*UNSPECIFIED_OPERATING_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*UNSPECIFIED_OPERATING_MODE* 0)
(intern "*MAINTENANCE_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*MAINTENANCE_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*MAINTENANCE_MODE* 1)
(intern "*UPDATE_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*UPDATE_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*UPDATE_MODE* 2)
(intern "*UPDATE_COMPLETED_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*UPDATE_COMPLETED_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*UPDATE_COMPLETED_MODE* 3)
(intern "*UPDATE_FAILED_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*UPDATE_FAILED_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*UPDATE_FAILED_MODE* 4)
(intern "*SHUTTING_DOWN_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*SHUTTING_DOWN_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*SHUTTING_DOWN_MODE* 5)
(intern "*RUN_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*RUN_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*RUN_MODE* 6)
(intern "*UPDATING_DEVICE_MODE*" (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(shadow '*UPDATING_DEVICE_MODE* (find-package "KORTEX_DRIVER::OPERATINGMODE"))
(defconstant kortex_driver::OperatingMode::*UPDATING_DEVICE_MODE* 7)

(defun kortex_driver::OperatingMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::OperatingMode::*UNSPECIFIED_OPERATING_MODE*)
        ((= const 1) 'kortex_driver::OperatingMode::*MAINTENANCE_MODE*)
        ((= const 2) 'kortex_driver::OperatingMode::*UPDATE_MODE*)
        ((= const 3) 'kortex_driver::OperatingMode::*UPDATE_COMPLETED_MODE*)
        ((= const 4) 'kortex_driver::OperatingMode::*UPDATE_FAILED_MODE*)
        ((= const 5) 'kortex_driver::OperatingMode::*SHUTTING_DOWN_MODE*)
        ((= const 6) 'kortex_driver::OperatingMode::*RUN_MODE*)
        ((= const 7) 'kortex_driver::OperatingMode::*UPDATING_DEVICE_MODE*)
        (t nil)))

(defclass kortex_driver::OperatingMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::OperatingMode
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

(setf (get kortex_driver::OperatingMode :md5sum-) "5399510deee390e03ac867dbfd636f27")
(setf (get kortex_driver::OperatingMode :datatype-) "kortex_driver/OperatingMode")
(setf (get kortex_driver::OperatingMode :definition-)
      "
uint32 UNSPECIFIED_OPERATING_MODE = 0

uint32 MAINTENANCE_MODE = 1

uint32 UPDATE_MODE = 2

uint32 UPDATE_COMPLETED_MODE = 3

uint32 UPDATE_FAILED_MODE = 4

uint32 SHUTTING_DOWN_MODE = 5

uint32 RUN_MODE = 6

uint32 UPDATING_DEVICE_MODE = 7

")



(provide :kortex_driver/OperatingMode "5399510deee390e03ac867dbfd636f27")


