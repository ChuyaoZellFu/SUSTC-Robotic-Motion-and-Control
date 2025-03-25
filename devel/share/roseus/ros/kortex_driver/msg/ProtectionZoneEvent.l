;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ProtectionZoneEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ProtectionZoneEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT")
  (make-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))

(in-package "ROS")
;;//! \htmlinclude ProtectionZoneEvent.msg.html


(intern "*UNSPECIFIED_PROTECTION_ZONE_EVENT*" (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(shadow '*UNSPECIFIED_PROTECTION_ZONE_EVENT* (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(defconstant kortex_driver::ProtectionZoneEvent::*UNSPECIFIED_PROTECTION_ZONE_EVENT* 0)
(intern "*REACHED*" (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(shadow '*REACHED* (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(defconstant kortex_driver::ProtectionZoneEvent::*REACHED* 1)
(intern "*ENTERED*" (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(shadow '*ENTERED* (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(defconstant kortex_driver::ProtectionZoneEvent::*ENTERED* 2)
(intern "*EXITED*" (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(shadow '*EXITED* (find-package "KORTEX_DRIVER::PROTECTIONZONEEVENT"))
(defconstant kortex_driver::ProtectionZoneEvent::*EXITED* 3)

(defun kortex_driver::ProtectionZoneEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ProtectionZoneEvent::*UNSPECIFIED_PROTECTION_ZONE_EVENT*)
        ((= const 1) 'kortex_driver::ProtectionZoneEvent::*REACHED*)
        ((= const 2) 'kortex_driver::ProtectionZoneEvent::*ENTERED*)
        ((= const 3) 'kortex_driver::ProtectionZoneEvent::*EXITED*)
        (t nil)))

(defclass kortex_driver::ProtectionZoneEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::ProtectionZoneEvent
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

(setf (get kortex_driver::ProtectionZoneEvent :md5sum-) "9e1a0e125d0c3437f9fc79af445cf181")
(setf (get kortex_driver::ProtectionZoneEvent :datatype-) "kortex_driver/ProtectionZoneEvent")
(setf (get kortex_driver::ProtectionZoneEvent :definition-)
      "
uint32 UNSPECIFIED_PROTECTION_ZONE_EVENT = 0

uint32 REACHED = 1

uint32 ENTERED = 2

uint32 EXITED = 3

")



(provide :kortex_driver/ProtectionZoneEvent "9e1a0e125d0c3437f9fc79af445cf181")


