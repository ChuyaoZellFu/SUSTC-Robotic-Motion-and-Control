;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConfigurationNotificationEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConfigurationNotificationEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT")
  (make-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))

(in-package "ROS")
;;//! \htmlinclude ConfigurationNotificationEvent.msg.html


(intern "*UNSPECIFIED_CONFIGURATION_EVENT*" (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(shadow '*UNSPECIFIED_CONFIGURATION_EVENT* (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(defconstant kortex_driver::ConfigurationNotificationEvent::*UNSPECIFIED_CONFIGURATION_EVENT* 0)
(intern "*CONFIGURATION_UPDATED*" (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(shadow '*CONFIGURATION_UPDATED* (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(defconstant kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_UPDATED* 1)
(intern "*CONFIGURATION_DELETED*" (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(shadow '*CONFIGURATION_DELETED* (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(defconstant kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_DELETED* 2)
(intern "*CONFIGURATION_DELETED_ALL*" (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(shadow '*CONFIGURATION_DELETED_ALL* (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(defconstant kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_DELETED_ALL* 3)
(intern "*CONFIGURATION_CREATED*" (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(shadow '*CONFIGURATION_CREATED* (find-package "KORTEX_DRIVER::CONFIGURATIONNOTIFICATIONEVENT"))
(defconstant kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_CREATED* 4)

(defun kortex_driver::ConfigurationNotificationEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ConfigurationNotificationEvent::*UNSPECIFIED_CONFIGURATION_EVENT*)
        ((= const 1) 'kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_UPDATED*)
        ((= const 2) 'kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_DELETED*)
        ((= const 3) 'kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_DELETED_ALL*)
        ((= const 4) 'kortex_driver::ConfigurationNotificationEvent::*CONFIGURATION_CREATED*)
        (t nil)))

(defclass kortex_driver::ConfigurationNotificationEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::ConfigurationNotificationEvent
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

(setf (get kortex_driver::ConfigurationNotificationEvent :md5sum-) "c8a6a5a55ab6b7b3399179fb55277b67")
(setf (get kortex_driver::ConfigurationNotificationEvent :datatype-) "kortex_driver/ConfigurationNotificationEvent")
(setf (get kortex_driver::ConfigurationNotificationEvent :definition-)
      "
uint32 UNSPECIFIED_CONFIGURATION_EVENT = 0

uint32 CONFIGURATION_UPDATED = 1

uint32 CONFIGURATION_DELETED = 2

uint32 CONFIGURATION_DELETED_ALL = 3

uint32 CONFIGURATION_CREATED = 4

")



(provide :kortex_driver/ConfigurationNotificationEvent "c8a6a5a55ab6b7b3399179fb55277b67")


