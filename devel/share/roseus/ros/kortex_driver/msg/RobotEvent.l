;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RobotEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RobotEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ROBOTEVENT")
  (make-package "KORTEX_DRIVER::ROBOTEVENT"))

(in-package "ROS")
;;//! \htmlinclude RobotEvent.msg.html


(intern "*UNSPECIFIED_ROBOT_EVENT*" (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(shadow '*UNSPECIFIED_ROBOT_EVENT* (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(defconstant kortex_driver::RobotEvent::*UNSPECIFIED_ROBOT_EVENT* 0)
(intern "*ARM_CONNECTED*" (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(shadow '*ARM_CONNECTED* (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(defconstant kortex_driver::RobotEvent::*ARM_CONNECTED* 1)
(intern "*ARM_DISCONNECTED*" (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(shadow '*ARM_DISCONNECTED* (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(defconstant kortex_driver::RobotEvent::*ARM_DISCONNECTED* 2)
(intern "*TOOL_CONNECTED*" (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(shadow '*TOOL_CONNECTED* (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(defconstant kortex_driver::RobotEvent::*TOOL_CONNECTED* 5)
(intern "*TOOL_DISCONNECTED*" (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(shadow '*TOOL_DISCONNECTED* (find-package "KORTEX_DRIVER::ROBOTEVENT"))
(defconstant kortex_driver::RobotEvent::*TOOL_DISCONNECTED* 6)

(defun kortex_driver::RobotEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::RobotEvent::*UNSPECIFIED_ROBOT_EVENT*)
        ((= const 1) 'kortex_driver::RobotEvent::*ARM_CONNECTED*)
        ((= const 2) 'kortex_driver::RobotEvent::*ARM_DISCONNECTED*)
        ((= const 5) 'kortex_driver::RobotEvent::*TOOL_CONNECTED*)
        ((= const 6) 'kortex_driver::RobotEvent::*TOOL_DISCONNECTED*)
        (t nil)))

(defclass kortex_driver::RobotEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::RobotEvent
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

(setf (get kortex_driver::RobotEvent :md5sum-) "7edc4e787111520cc97803b60a159bc5")
(setf (get kortex_driver::RobotEvent :datatype-) "kortex_driver/RobotEvent")
(setf (get kortex_driver::RobotEvent :definition-)
      "
uint32 UNSPECIFIED_ROBOT_EVENT = 0

uint32 ARM_CONNECTED = 1

uint32 ARM_DISCONNECTED = 2

uint32 TOOL_CONNECTED = 5

uint32 TOOL_DISCONNECTED = 6

")



(provide :kortex_driver/RobotEvent "7edc4e787111520cc97803b60a159bc5")


