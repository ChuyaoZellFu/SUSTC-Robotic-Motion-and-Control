;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TrajectoryContinuityMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TrajectoryContinuityMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE")
  (make-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryContinuityMode.msg.html


(intern "*TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(shadow '*TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED* (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(defconstant kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED* 0)
(intern "*TRAJECTORY_CONTINUITY_MODE_POSITION*" (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(shadow '*TRAJECTORY_CONTINUITY_MODE_POSITION* (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(defconstant kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_POSITION* 1)
(intern "*TRAJECTORY_CONTINUITY_MODE_SPEED*" (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(shadow '*TRAJECTORY_CONTINUITY_MODE_SPEED* (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(defconstant kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_SPEED* 2)
(intern "*TRAJECTORY_CONTINUITY_MODE_ACCELERATION*" (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(shadow '*TRAJECTORY_CONTINUITY_MODE_ACCELERATION* (find-package "KORTEX_DRIVER::TRAJECTORYCONTINUITYMODE"))
(defconstant kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_ACCELERATION* 3)

(defun kortex_driver::TrajectoryContinuityMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_POSITION*)
        ((= const 2) 'kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_SPEED*)
        ((= const 3) 'kortex_driver::TrajectoryContinuityMode::*TRAJECTORY_CONTINUITY_MODE_ACCELERATION*)
        (t nil)))

(defclass kortex_driver::TrajectoryContinuityMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::TrajectoryContinuityMode
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

(setf (get kortex_driver::TrajectoryContinuityMode :md5sum-) "a762cc9649feca6c4672c7308715b451")
(setf (get kortex_driver::TrajectoryContinuityMode :datatype-) "kortex_driver/TrajectoryContinuityMode")
(setf (get kortex_driver::TrajectoryContinuityMode :definition-)
      "
uint32 TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED = 0

uint32 TRAJECTORY_CONTINUITY_MODE_POSITION = 1

uint32 TRAJECTORY_CONTINUITY_MODE_SPEED = 2

uint32 TRAJECTORY_CONTINUITY_MODE_ACCELERATION = 3

")



(provide :kortex_driver/TrajectoryContinuityMode "a762cc9649feca6c4672c7308715b451")


