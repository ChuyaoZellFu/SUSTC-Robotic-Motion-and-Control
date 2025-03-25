;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerEventType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerEventType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE")
  (make-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE"))

(in-package "ROS")
;;//! \htmlinclude ControllerEventType.msg.html


(intern "*UNSPECIFIED_CONTROLLER_EVENT*" (find-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE"))
(shadow '*UNSPECIFIED_CONTROLLER_EVENT* (find-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE"))
(defconstant kortex_driver::ControllerEventType::*UNSPECIFIED_CONTROLLER_EVENT* 0)
(intern "*CONTROLLER_DISCONNECTED*" (find-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE"))
(shadow '*CONTROLLER_DISCONNECTED* (find-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE"))
(defconstant kortex_driver::ControllerEventType::*CONTROLLER_DISCONNECTED* 1)
(intern "*CONTROLLER_CONNECTED*" (find-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE"))
(shadow '*CONTROLLER_CONNECTED* (find-package "KORTEX_DRIVER::CONTROLLEREVENTTYPE"))
(defconstant kortex_driver::ControllerEventType::*CONTROLLER_CONNECTED* 2)

(defun kortex_driver::ControllerEventType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ControllerEventType::*UNSPECIFIED_CONTROLLER_EVENT*)
        ((= const 1) 'kortex_driver::ControllerEventType::*CONTROLLER_DISCONNECTED*)
        ((= const 2) 'kortex_driver::ControllerEventType::*CONTROLLER_CONNECTED*)
        (t nil)))

(defclass kortex_driver::ControllerEventType
  :super ros::object
  :slots ())

(defmethod kortex_driver::ControllerEventType
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

(setf (get kortex_driver::ControllerEventType :md5sum-) "4bffb5259aaf49860456ce038da78405")
(setf (get kortex_driver::ControllerEventType :datatype-) "kortex_driver/ControllerEventType")
(setf (get kortex_driver::ControllerEventType :definition-)
      "
uint32 UNSPECIFIED_CONTROLLER_EVENT = 0

uint32 CONTROLLER_DISCONNECTED = 1

uint32 CONTROLLER_CONNECTED = 2

")



(provide :kortex_driver/ControllerEventType "4bffb5259aaf49860456ce038da78405")


