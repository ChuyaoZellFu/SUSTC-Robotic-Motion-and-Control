;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GripperMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GripperMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPERMODE")
  (make-package "KORTEX_DRIVER::GRIPPERMODE"))

(in-package "ROS")
;;//! \htmlinclude GripperMode.msg.html


(intern "*UNSPECIFIED_GRIPPER_MODE*" (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(shadow '*UNSPECIFIED_GRIPPER_MODE* (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(defconstant kortex_driver::GripperMode::*UNSPECIFIED_GRIPPER_MODE* 0)
(intern "*GRIPPER_FORCE*" (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(shadow '*GRIPPER_FORCE* (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(defconstant kortex_driver::GripperMode::*GRIPPER_FORCE* 1)
(intern "*GRIPPER_SPEED*" (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(shadow '*GRIPPER_SPEED* (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(defconstant kortex_driver::GripperMode::*GRIPPER_SPEED* 2)
(intern "*GRIPPER_POSITION*" (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(shadow '*GRIPPER_POSITION* (find-package "KORTEX_DRIVER::GRIPPERMODE"))
(defconstant kortex_driver::GripperMode::*GRIPPER_POSITION* 3)

(defun kortex_driver::GripperMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GripperMode::*UNSPECIFIED_GRIPPER_MODE*)
        ((= const 1) 'kortex_driver::GripperMode::*GRIPPER_FORCE*)
        ((= const 2) 'kortex_driver::GripperMode::*GRIPPER_SPEED*)
        ((= const 3) 'kortex_driver::GripperMode::*GRIPPER_POSITION*)
        (t nil)))

(defclass kortex_driver::GripperMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::GripperMode
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

(setf (get kortex_driver::GripperMode :md5sum-) "91237e67935408a1b67c1d3e83f53d6b")
(setf (get kortex_driver::GripperMode :datatype-) "kortex_driver/GripperMode")
(setf (get kortex_driver::GripperMode :definition-)
      "
uint32 UNSPECIFIED_GRIPPER_MODE = 0

uint32 GRIPPER_FORCE = 1

uint32 GRIPPER_SPEED = 2

uint32 GRIPPER_POSITION = 3

")



(provide :kortex_driver/GripperMode "91237e67935408a1b67c1d3e83f53d6b")


