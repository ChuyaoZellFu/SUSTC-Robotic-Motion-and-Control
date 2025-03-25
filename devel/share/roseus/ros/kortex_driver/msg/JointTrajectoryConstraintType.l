;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointTrajectoryConstraintType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointTrajectoryConstraintType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE")
  (make-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE"))

(in-package "ROS")
;;//! \htmlinclude JointTrajectoryConstraintType.msg.html


(intern "*UNSPECIFIED_JOINT_CONSTRAINT*" (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE"))
(shadow '*UNSPECIFIED_JOINT_CONSTRAINT* (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE"))
(defconstant kortex_driver::JointTrajectoryConstraintType::*UNSPECIFIED_JOINT_CONSTRAINT* 0)
(intern "*JOINT_CONSTRAINT_DURATION*" (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE"))
(shadow '*JOINT_CONSTRAINT_DURATION* (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE"))
(defconstant kortex_driver::JointTrajectoryConstraintType::*JOINT_CONSTRAINT_DURATION* 1)
(intern "*JOINT_CONSTRAINT_SPEED*" (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE"))
(shadow '*JOINT_CONSTRAINT_SPEED* (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINTTYPE"))
(defconstant kortex_driver::JointTrajectoryConstraintType::*JOINT_CONSTRAINT_SPEED* 2)

(defun kortex_driver::JointTrajectoryConstraintType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::JointTrajectoryConstraintType::*UNSPECIFIED_JOINT_CONSTRAINT*)
        ((= const 1) 'kortex_driver::JointTrajectoryConstraintType::*JOINT_CONSTRAINT_DURATION*)
        ((= const 2) 'kortex_driver::JointTrajectoryConstraintType::*JOINT_CONSTRAINT_SPEED*)
        (t nil)))

(defclass kortex_driver::JointTrajectoryConstraintType
  :super ros::object
  :slots ())

(defmethod kortex_driver::JointTrajectoryConstraintType
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

(setf (get kortex_driver::JointTrajectoryConstraintType :md5sum-) "a007b68cae061ff813f21ddba74d4035")
(setf (get kortex_driver::JointTrajectoryConstraintType :datatype-) "kortex_driver/JointTrajectoryConstraintType")
(setf (get kortex_driver::JointTrajectoryConstraintType :definition-)
      "
uint32 UNSPECIFIED_JOINT_CONSTRAINT = 0

uint32 JOINT_CONSTRAINT_DURATION = 1

uint32 JOINT_CONSTRAINT_SPEED = 2

")



(provide :kortex_driver/JointTrajectoryConstraintType "a007b68cae061ff813f21ddba74d4035")


