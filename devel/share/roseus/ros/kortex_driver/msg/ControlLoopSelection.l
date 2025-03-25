;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlLoopSelection)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlLoopSelection (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION")
  (make-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))

(in-package "ROS")
;;//! \htmlinclude ControlLoopSelection.msg.html


(intern "*RESERVED*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*RESERVED* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*RESERVED* 0)
(intern "*JOINT_POSITION*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*JOINT_POSITION* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*JOINT_POSITION* 1)
(intern "*MOTOR_POSITION*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*MOTOR_POSITION* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*MOTOR_POSITION* 2)
(intern "*JOINT_VELOCITY*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*JOINT_VELOCITY* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*JOINT_VELOCITY* 4)
(intern "*MOTOR_VELOCITY*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*MOTOR_VELOCITY* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*MOTOR_VELOCITY* 8)
(intern "*JOINT_TORQUE*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*JOINT_TORQUE* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*JOINT_TORQUE* 16)
(intern "*MOTOR_CURRENT*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*MOTOR_CURRENT* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*MOTOR_CURRENT* 32)
(intern "*JOINT_TORQUE_HIGH_VELOCITY*" (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(shadow '*JOINT_TORQUE_HIGH_VELOCITY* (find-package "KORTEX_DRIVER::CONTROLLOOPSELECTION"))
(defconstant kortex_driver::ControlLoopSelection::*JOINT_TORQUE_HIGH_VELOCITY* 64)

(defun kortex_driver::ControlLoopSelection-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ControlLoopSelection::*RESERVED*)
        ((= const 1) 'kortex_driver::ControlLoopSelection::*JOINT_POSITION*)
        ((= const 2) 'kortex_driver::ControlLoopSelection::*MOTOR_POSITION*)
        ((= const 4) 'kortex_driver::ControlLoopSelection::*JOINT_VELOCITY*)
        ((= const 8) 'kortex_driver::ControlLoopSelection::*MOTOR_VELOCITY*)
        ((= const 16) 'kortex_driver::ControlLoopSelection::*JOINT_TORQUE*)
        ((= const 32) 'kortex_driver::ControlLoopSelection::*MOTOR_CURRENT*)
        ((= const 64) 'kortex_driver::ControlLoopSelection::*JOINT_TORQUE_HIGH_VELOCITY*)
        (t nil)))

(defclass kortex_driver::ControlLoopSelection
  :super ros::object
  :slots ())

(defmethod kortex_driver::ControlLoopSelection
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

(setf (get kortex_driver::ControlLoopSelection :md5sum-) "85d717df01106b4f27c7c235a5980355")
(setf (get kortex_driver::ControlLoopSelection :datatype-) "kortex_driver/ControlLoopSelection")
(setf (get kortex_driver::ControlLoopSelection :definition-)
      "
uint32 RESERVED = 0

uint32 JOINT_POSITION = 1

uint32 MOTOR_POSITION = 2

uint32 JOINT_VELOCITY = 4

uint32 MOTOR_VELOCITY = 8

uint32 JOINT_TORQUE = 16

uint32 MOTOR_CURRENT = 32

uint32 JOINT_TORQUE_HIGH_VELOCITY = 64

")



(provide :kortex_driver/ControlLoopSelection "85d717df01106b4f27c7c235a5980355")


