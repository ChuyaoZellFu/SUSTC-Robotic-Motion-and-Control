;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TrajectoryErrorIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TrajectoryErrorIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER")
  (make-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryErrorIdentifier.msg.html


(intern "*TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED*" (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(shadow '*TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED* (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(defconstant kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED* 0)
(intern "*TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE*" (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(shadow '*TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE* (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(defconstant kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE* 1)
(intern "*TRAJECTORY_ERROR_IDENTIFIER_TIME*" (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(shadow '*TRAJECTORY_ERROR_IDENTIFIER_TIME* (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(defconstant kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_TIME* 2)
(intern "*TRAJECTORY_ERROR_IDENTIFIER_POSITION*" (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(shadow '*TRAJECTORY_ERROR_IDENTIFIER_POSITION* (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(defconstant kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_POSITION* 3)
(intern "*TRAJECTORY_ERROR_IDENTIFIER_VELOCITY*" (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(shadow '*TRAJECTORY_ERROR_IDENTIFIER_VELOCITY* (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(defconstant kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_VELOCITY* 4)
(intern "*TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION*" (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(shadow '*TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION* (find-package "KORTEX_DRIVER::TRAJECTORYERRORIDENTIFIER"))
(defconstant kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION* 5)

(defun kortex_driver::TrajectoryErrorIdentifier-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE*)
        ((= const 2) 'kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_TIME*)
        ((= const 3) 'kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_POSITION*)
        ((= const 4) 'kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_VELOCITY*)
        ((= const 5) 'kortex_driver::TrajectoryErrorIdentifier::*TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION*)
        (t nil)))

(defclass kortex_driver::TrajectoryErrorIdentifier
  :super ros::object
  :slots ())

(defmethod kortex_driver::TrajectoryErrorIdentifier
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

(setf (get kortex_driver::TrajectoryErrorIdentifier :md5sum-) "d64304d18e3e5294df54e8c5ef2c83c9")
(setf (get kortex_driver::TrajectoryErrorIdentifier :datatype-) "kortex_driver/TrajectoryErrorIdentifier")
(setf (get kortex_driver::TrajectoryErrorIdentifier :definition-)
      "
uint32 TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED = 0

uint32 TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE = 1

uint32 TRAJECTORY_ERROR_IDENTIFIER_TIME = 2

uint32 TRAJECTORY_ERROR_IDENTIFIER_POSITION = 3

uint32 TRAJECTORY_ERROR_IDENTIFIER_VELOCITY = 4

uint32 TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION = 5

")



(provide :kortex_driver/TrajectoryErrorIdentifier "d64304d18e3e5294df54e8c5ef2c83c9")


