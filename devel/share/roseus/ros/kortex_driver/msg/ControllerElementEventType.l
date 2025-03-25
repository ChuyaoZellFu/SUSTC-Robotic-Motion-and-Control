;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerElementEventType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerElementEventType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE")
  (make-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))

(in-package "ROS")
;;//! \htmlinclude ControllerElementEventType.msg.html


(intern "*UNSPECIFIED_CONTROLLER_ELEMENT_EVENT*" (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(shadow '*UNSPECIFIED_CONTROLLER_ELEMENT_EVENT* (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(defconstant kortex_driver::ControllerElementEventType::*UNSPECIFIED_CONTROLLER_ELEMENT_EVENT* 0)
(intern "*AXIS_MOVED*" (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(shadow '*AXIS_MOVED* (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(defconstant kortex_driver::ControllerElementEventType::*AXIS_MOVED* 1)
(intern "*BUTTON_DOWN*" (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(shadow '*BUTTON_DOWN* (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(defconstant kortex_driver::ControllerElementEventType::*BUTTON_DOWN* 2)
(intern "*BUTTON_UP*" (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(shadow '*BUTTON_UP* (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(defconstant kortex_driver::ControllerElementEventType::*BUTTON_UP* 3)
(intern "*BUTTON_CLICK*" (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(shadow '*BUTTON_CLICK* (find-package "KORTEX_DRIVER::CONTROLLERELEMENTEVENTTYPE"))
(defconstant kortex_driver::ControllerElementEventType::*BUTTON_CLICK* 4)

(defun kortex_driver::ControllerElementEventType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ControllerElementEventType::*UNSPECIFIED_CONTROLLER_ELEMENT_EVENT*)
        ((= const 1) 'kortex_driver::ControllerElementEventType::*AXIS_MOVED*)
        ((= const 2) 'kortex_driver::ControllerElementEventType::*BUTTON_DOWN*)
        ((= const 3) 'kortex_driver::ControllerElementEventType::*BUTTON_UP*)
        ((= const 4) 'kortex_driver::ControllerElementEventType::*BUTTON_CLICK*)
        (t nil)))

(defclass kortex_driver::ControllerElementEventType
  :super ros::object
  :slots ())

(defmethod kortex_driver::ControllerElementEventType
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

(setf (get kortex_driver::ControllerElementEventType :md5sum-) "30410419f52cbfa760b386b2213e4309")
(setf (get kortex_driver::ControllerElementEventType :datatype-) "kortex_driver/ControllerElementEventType")
(setf (get kortex_driver::ControllerElementEventType :definition-)
      "
uint32 UNSPECIFIED_CONTROLLER_ELEMENT_EVENT = 0

uint32 AXIS_MOVED = 1

uint32 BUTTON_DOWN = 2

uint32 BUTTON_UP = 3

uint32 BUTTON_CLICK = 4

")



(provide :kortex_driver/ControllerElementEventType "30410419f52cbfa760b386b2213e4309")


