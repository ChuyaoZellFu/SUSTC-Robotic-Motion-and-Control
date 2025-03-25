;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ArmState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ArmState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ARMSTATE")
  (make-package "KORTEX_DRIVER::ARMSTATE"))

(in-package "ROS")
;;//! \htmlinclude ArmState.msg.html


(intern "*ARMSTATE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_UNSPECIFIED* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_UNSPECIFIED* 0)
(intern "*ARMSTATE_BASE_INITIALIZATION*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_BASE_INITIALIZATION* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_BASE_INITIALIZATION* 1)
(intern "*ARMSTATE_IDLE*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_IDLE* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_IDLE* 2)
(intern "*ARMSTATE_INITIALIZATION*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_INITIALIZATION* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_INITIALIZATION* 3)
(intern "*ARMSTATE_IN_FAULT*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_IN_FAULT* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_IN_FAULT* 4)
(intern "*ARMSTATE_MAINTENANCE*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_MAINTENANCE* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_MAINTENANCE* 5)
(intern "*ARMSTATE_SERVOING_LOW_LEVEL*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_SERVOING_LOW_LEVEL* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_SERVOING_LOW_LEVEL* 6)
(intern "*ARMSTATE_SERVOING_READY*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_SERVOING_READY* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_SERVOING_READY* 7)
(intern "*ARMSTATE_SERVOING_PLAYING_SEQUENCE*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_SERVOING_PLAYING_SEQUENCE* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_SERVOING_PLAYING_SEQUENCE* 8)
(intern "*ARMSTATE_SERVOING_MANUALLY_CONTROLLED*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_SERVOING_MANUALLY_CONTROLLED* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_SERVOING_MANUALLY_CONTROLLED* 9)
(intern "*ARMSTATE_BRAKE_RELEASING*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_BRAKE_RELEASING* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_BRAKE_RELEASING* 10)
(intern "*ARMSTATE_RESERVED*" (find-package "KORTEX_DRIVER::ARMSTATE"))
(shadow '*ARMSTATE_RESERVED* (find-package "KORTEX_DRIVER::ARMSTATE"))
(defconstant kortex_driver::ArmState::*ARMSTATE_RESERVED* 255)

(defun kortex_driver::ArmState-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ArmState::*ARMSTATE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::ArmState::*ARMSTATE_BASE_INITIALIZATION*)
        ((= const 2) 'kortex_driver::ArmState::*ARMSTATE_IDLE*)
        ((= const 3) 'kortex_driver::ArmState::*ARMSTATE_INITIALIZATION*)
        ((= const 4) 'kortex_driver::ArmState::*ARMSTATE_IN_FAULT*)
        ((= const 5) 'kortex_driver::ArmState::*ARMSTATE_MAINTENANCE*)
        ((= const 6) 'kortex_driver::ArmState::*ARMSTATE_SERVOING_LOW_LEVEL*)
        ((= const 7) 'kortex_driver::ArmState::*ARMSTATE_SERVOING_READY*)
        ((= const 8) 'kortex_driver::ArmState::*ARMSTATE_SERVOING_PLAYING_SEQUENCE*)
        ((= const 9) 'kortex_driver::ArmState::*ARMSTATE_SERVOING_MANUALLY_CONTROLLED*)
        ((= const 10) 'kortex_driver::ArmState::*ARMSTATE_BRAKE_RELEASING*)
        ((= const 255) 'kortex_driver::ArmState::*ARMSTATE_RESERVED*)
        (t nil)))

(defclass kortex_driver::ArmState
  :super ros::object
  :slots ())

(defmethod kortex_driver::ArmState
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

(setf (get kortex_driver::ArmState :md5sum-) "ac681740837f166518d3ae6a4e6eeda1")
(setf (get kortex_driver::ArmState :datatype-) "kortex_driver/ArmState")
(setf (get kortex_driver::ArmState :definition-)
      "
uint32 ARMSTATE_UNSPECIFIED = 0

uint32 ARMSTATE_BASE_INITIALIZATION = 1

uint32 ARMSTATE_IDLE = 2

uint32 ARMSTATE_INITIALIZATION = 3

uint32 ARMSTATE_IN_FAULT = 4

uint32 ARMSTATE_MAINTENANCE = 5

uint32 ARMSTATE_SERVOING_LOW_LEVEL = 6

uint32 ARMSTATE_SERVOING_READY = 7

uint32 ARMSTATE_SERVOING_PLAYING_SEQUENCE = 8

uint32 ARMSTATE_SERVOING_MANUALLY_CONTROLLED = 9

uint32 ARMSTATE_BRAKE_RELEASING = 10

uint32 ARMSTATE_RESERVED = 255

")



(provide :kortex_driver/ArmState "ac681740837f166518d3ae6a4e6eeda1")


