;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SnapshotType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SnapshotType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SNAPSHOTTYPE")
  (make-package "KORTEX_DRIVER::SNAPSHOTTYPE"))

(in-package "ROS")
;;//! \htmlinclude SnapshotType.msg.html


(intern "*UNSPECIFIED_SNAPSHOT*" (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(shadow '*UNSPECIFIED_SNAPSHOT* (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(defconstant kortex_driver::SnapshotType::*UNSPECIFIED_SNAPSHOT* 0)
(intern "*CARTESIAN_POSITION_SNAPSHOT*" (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(shadow '*CARTESIAN_POSITION_SNAPSHOT* (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(defconstant kortex_driver::SnapshotType::*CARTESIAN_POSITION_SNAPSHOT* 1)
(intern "*JOINT_POSITION_SNAPSHOT*" (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(shadow '*JOINT_POSITION_SNAPSHOT* (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(defconstant kortex_driver::SnapshotType::*JOINT_POSITION_SNAPSHOT* 2)
(intern "*GRIPPER_SNAPSHOT*" (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(shadow '*GRIPPER_SNAPSHOT* (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(defconstant kortex_driver::SnapshotType::*GRIPPER_SNAPSHOT* 3)
(intern "*COMBINED_SNAPSHOT*" (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(shadow '*COMBINED_SNAPSHOT* (find-package "KORTEX_DRIVER::SNAPSHOTTYPE"))
(defconstant kortex_driver::SnapshotType::*COMBINED_SNAPSHOT* 4)

(defun kortex_driver::SnapshotType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::SnapshotType::*UNSPECIFIED_SNAPSHOT*)
        ((= const 1) 'kortex_driver::SnapshotType::*CARTESIAN_POSITION_SNAPSHOT*)
        ((= const 2) 'kortex_driver::SnapshotType::*JOINT_POSITION_SNAPSHOT*)
        ((= const 3) 'kortex_driver::SnapshotType::*GRIPPER_SNAPSHOT*)
        ((= const 4) 'kortex_driver::SnapshotType::*COMBINED_SNAPSHOT*)
        (t nil)))

(defclass kortex_driver::SnapshotType
  :super ros::object
  :slots ())

(defmethod kortex_driver::SnapshotType
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

(setf (get kortex_driver::SnapshotType :md5sum-) "a9efee68393bf09e7b23d3b042a31552")
(setf (get kortex_driver::SnapshotType :datatype-) "kortex_driver/SnapshotType")
(setf (get kortex_driver::SnapshotType :definition-)
      "
uint32 UNSPECIFIED_SNAPSHOT = 0

uint32 CARTESIAN_POSITION_SNAPSHOT = 1

uint32 JOINT_POSITION_SNAPSHOT = 2

uint32 GRIPPER_SNAPSHOT = 3

uint32 COMBINED_SNAPSHOT = 4

")



(provide :kortex_driver/SnapshotType "a9efee68393bf09e7b23d3b042a31552")


