;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::NotificationType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'NotificationType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE")
  (make-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))

(in-package "ROS")
;;//! \htmlinclude NotificationType.msg.html


(intern "*NOTIFICATION_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(shadow '*NOTIFICATION_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(defconstant kortex_driver::NotificationType::*NOTIFICATION_TYPE_UNSPECIFIED* 0)
(intern "*NOTIFICATION_TYPE_THRESHOLD*" (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(shadow '*NOTIFICATION_TYPE_THRESHOLD* (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(defconstant kortex_driver::NotificationType::*NOTIFICATION_TYPE_THRESHOLD* 1)
(intern "*NOTIFICATION_TYPE_FIX_RATE*" (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(shadow '*NOTIFICATION_TYPE_FIX_RATE* (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(defconstant kortex_driver::NotificationType::*NOTIFICATION_TYPE_FIX_RATE* 2)
(intern "*NOTIFICATION_TYPE_EVENT*" (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(shadow '*NOTIFICATION_TYPE_EVENT* (find-package "KORTEX_DRIVER::NOTIFICATIONTYPE"))
(defconstant kortex_driver::NotificationType::*NOTIFICATION_TYPE_EVENT* 3)

(defun kortex_driver::NotificationType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::NotificationType::*NOTIFICATION_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::NotificationType::*NOTIFICATION_TYPE_THRESHOLD*)
        ((= const 2) 'kortex_driver::NotificationType::*NOTIFICATION_TYPE_FIX_RATE*)
        ((= const 3) 'kortex_driver::NotificationType::*NOTIFICATION_TYPE_EVENT*)
        (t nil)))

(defclass kortex_driver::NotificationType
  :super ros::object
  :slots ())

(defmethod kortex_driver::NotificationType
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

(setf (get kortex_driver::NotificationType :md5sum-) "9d8153f0fe98641698596673829b2649")
(setf (get kortex_driver::NotificationType :datatype-) "kortex_driver/NotificationType")
(setf (get kortex_driver::NotificationType :definition-)
      "
uint32 NOTIFICATION_TYPE_UNSPECIFIED = 0

uint32 NOTIFICATION_TYPE_THRESHOLD = 1

uint32 NOTIFICATION_TYPE_FIX_RATE = 2

uint32 NOTIFICATION_TYPE_EVENT = 3

")



(provide :kortex_driver/NotificationType "9d8153f0fe98641698596673829b2649")


