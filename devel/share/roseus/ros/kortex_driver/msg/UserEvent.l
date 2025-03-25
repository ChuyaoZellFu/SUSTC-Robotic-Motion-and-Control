;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UserEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UserEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::USEREVENT")
  (make-package "KORTEX_DRIVER::USEREVENT"))

(in-package "ROS")
;;//! \htmlinclude UserEvent.msg.html


(intern "*UNSPECIFIED_USER_EVENT*" (find-package "KORTEX_DRIVER::USEREVENT"))
(shadow '*UNSPECIFIED_USER_EVENT* (find-package "KORTEX_DRIVER::USEREVENT"))
(defconstant kortex_driver::UserEvent::*UNSPECIFIED_USER_EVENT* 0)
(intern "*LOGGED_OUT*" (find-package "KORTEX_DRIVER::USEREVENT"))
(shadow '*LOGGED_OUT* (find-package "KORTEX_DRIVER::USEREVENT"))
(defconstant kortex_driver::UserEvent::*LOGGED_OUT* 1)
(intern "*LOGGED_IN*" (find-package "KORTEX_DRIVER::USEREVENT"))
(shadow '*LOGGED_IN* (find-package "KORTEX_DRIVER::USEREVENT"))
(defconstant kortex_driver::UserEvent::*LOGGED_IN* 2)

(defun kortex_driver::UserEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::UserEvent::*UNSPECIFIED_USER_EVENT*)
        ((= const 1) 'kortex_driver::UserEvent::*LOGGED_OUT*)
        ((= const 2) 'kortex_driver::UserEvent::*LOGGED_IN*)
        (t nil)))

(defclass kortex_driver::UserEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::UserEvent
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

(setf (get kortex_driver::UserEvent :md5sum-) "6b6b55b08fd45b34a43cf1d05e2fdbdc")
(setf (get kortex_driver::UserEvent :datatype-) "kortex_driver/UserEvent")
(setf (get kortex_driver::UserEvent :definition-)
      "
uint32 UNSPECIFIED_USER_EVENT = 0

uint32 LOGGED_OUT = 1

uint32 LOGGED_IN = 2

")



(provide :kortex_driver/UserEvent "6b6b55b08fd45b34a43cf1d05e2fdbdc")


