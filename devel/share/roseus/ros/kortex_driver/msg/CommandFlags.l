;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CommandFlags)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CommandFlags (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COMMANDFLAGS")
  (make-package "KORTEX_DRIVER::COMMANDFLAGS"))

(in-package "ROS")
;;//! \htmlinclude CommandFlags.msg.html


(intern "*NO_COMMAND*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*NO_COMMAND* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*NO_COMMAND* 0)
(intern "*SERVO_ENABLE*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*SERVO_ENABLE* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*SERVO_ENABLE* 1)
(intern "*BRAKE_DISABLE*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*BRAKE_DISABLE* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*BRAKE_DISABLE* 2)
(intern "*CLEAR_MAJOR_FAULT*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*CLEAR_MAJOR_FAULT* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*CLEAR_MAJOR_FAULT* 4)
(intern "*CLEAR_MINOR_FAULT*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*CLEAR_MINOR_FAULT* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*CLEAR_MINOR_FAULT* 8)
(intern "*PROTECTIVE_STOP*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*PROTECTIVE_STOP* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*PROTECTIVE_STOP* 16)
(intern "*FORCE_BRAKE_RELEASE*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*FORCE_BRAKE_RELEASE* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*FORCE_BRAKE_RELEASE* 32)
(intern "*IGNORE*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*IGNORE* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*IGNORE* 64)
(intern "*LOW_GAINS*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*LOW_GAINS* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*LOW_GAINS* 128)
(intern "*LED_0*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*LED_0* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*LED_0* 256)
(intern "*LED_1*" (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(shadow '*LED_1* (find-package "KORTEX_DRIVER::COMMANDFLAGS"))
(defconstant kortex_driver::CommandFlags::*LED_1* 512)

(defun kortex_driver::CommandFlags-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::CommandFlags::*NO_COMMAND*)
        ((= const 1) 'kortex_driver::CommandFlags::*SERVO_ENABLE*)
        ((= const 2) 'kortex_driver::CommandFlags::*BRAKE_DISABLE*)
        ((= const 4) 'kortex_driver::CommandFlags::*CLEAR_MAJOR_FAULT*)
        ((= const 8) 'kortex_driver::CommandFlags::*CLEAR_MINOR_FAULT*)
        ((= const 16) 'kortex_driver::CommandFlags::*PROTECTIVE_STOP*)
        ((= const 32) 'kortex_driver::CommandFlags::*FORCE_BRAKE_RELEASE*)
        ((= const 64) 'kortex_driver::CommandFlags::*IGNORE*)
        ((= const 128) 'kortex_driver::CommandFlags::*LOW_GAINS*)
        ((= const 256) 'kortex_driver::CommandFlags::*LED_0*)
        ((= const 512) 'kortex_driver::CommandFlags::*LED_1*)
        (t nil)))

(defclass kortex_driver::CommandFlags
  :super ros::object
  :slots ())

(defmethod kortex_driver::CommandFlags
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

(setf (get kortex_driver::CommandFlags :md5sum-) "e51ff7d8ad00c576e3aaa1ef0c879bf9")
(setf (get kortex_driver::CommandFlags :datatype-) "kortex_driver/CommandFlags")
(setf (get kortex_driver::CommandFlags :definition-)
      "
uint32 NO_COMMAND = 0

uint32 SERVO_ENABLE = 1

uint32 BRAKE_DISABLE = 2

uint32 CLEAR_MAJOR_FAULT = 4

uint32 CLEAR_MINOR_FAULT = 8

uint32 PROTECTIVE_STOP = 16

uint32 FORCE_BRAKE_RELEASE = 32

uint32 IGNORE = 64

uint32 LOW_GAINS = 128

uint32 LED_0 = 256

uint32 LED_1 = 512

")



(provide :kortex_driver/CommandFlags "e51ff7d8ad00c576e3aaa1ef0c879bf9")


