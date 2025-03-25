;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RobotiqGripperStatusFlags)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RobotiqGripperStatusFlags (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS")
  (make-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))

(in-package "ROS")
;;//! \htmlinclude RobotiqGripperStatusFlags.msg.html


(intern "*UNSPECIFIED_ROBOTIQ_STATUS*" (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(shadow '*UNSPECIFIED_ROBOTIQ_STATUS* (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(defconstant kortex_driver::RobotiqGripperStatusFlags::*UNSPECIFIED_ROBOTIQ_STATUS* 0)
(intern "*ROBOTIQ_STAT_INITIALIZED*" (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(shadow '*ROBOTIQ_STAT_INITIALIZED* (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(defconstant kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_INITIALIZED* 1)
(intern "*ROBOTIQ_STAT_OBJECT_DETECTED*" (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(shadow '*ROBOTIQ_STAT_OBJECT_DETECTED* (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(defconstant kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_OBJECT_DETECTED* 2)
(intern "*ROBOTIQ_STAT_POS_REACHED*" (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(shadow '*ROBOTIQ_STAT_POS_REACHED* (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(defconstant kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_POS_REACHED* 4)
(intern "*ROBOTIQ_STAT_STOPPED*" (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(shadow '*ROBOTIQ_STAT_STOPPED* (find-package "KORTEX_DRIVER::ROBOTIQGRIPPERSTATUSFLAGS"))
(defconstant kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_STOPPED* 8)

(defun kortex_driver::RobotiqGripperStatusFlags-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::RobotiqGripperStatusFlags::*UNSPECIFIED_ROBOTIQ_STATUS*)
        ((= const 1) 'kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_INITIALIZED*)
        ((= const 2) 'kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_OBJECT_DETECTED*)
        ((= const 4) 'kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_POS_REACHED*)
        ((= const 8) 'kortex_driver::RobotiqGripperStatusFlags::*ROBOTIQ_STAT_STOPPED*)
        (t nil)))

(defclass kortex_driver::RobotiqGripperStatusFlags
  :super ros::object
  :slots ())

(defmethod kortex_driver::RobotiqGripperStatusFlags
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

(setf (get kortex_driver::RobotiqGripperStatusFlags :md5sum-) "289d3fd6bd510e8e9380318bbf77ab70")
(setf (get kortex_driver::RobotiqGripperStatusFlags :datatype-) "kortex_driver/RobotiqGripperStatusFlags")
(setf (get kortex_driver::RobotiqGripperStatusFlags :definition-)
      "
uint32 UNSPECIFIED_ROBOTIQ_STATUS = 0

uint32 ROBOTIQ_STAT_INITIALIZED = 1

uint32 ROBOTIQ_STAT_OBJECT_DETECTED = 2

uint32 ROBOTIQ_STAT_POS_REACHED = 4

uint32 ROBOTIQ_STAT_STOPPED = 8

")



(provide :kortex_driver/RobotiqGripperStatusFlags "289d3fd6bd510e8e9380318bbf77ab70")


