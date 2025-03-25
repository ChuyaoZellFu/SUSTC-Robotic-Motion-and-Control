;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TrajectoryInfoType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TrajectoryInfoType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE")
  (make-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryInfoType.msg.html


(intern "*UNSPECIFIED_TRAJECTORY_INFORMATION*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*UNSPECIFIED_TRAJECTORY_INFORMATION* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*UNSPECIFIED_TRAJECTORY_INFORMATION* 0)
(intern "*JOINT_ACCELERATION_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*JOINT_ACCELERATION_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*JOINT_ACCELERATION_LIMIT_REACHED* 1)
(intern "*JOINT_SPEED_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*JOINT_SPEED_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*JOINT_SPEED_LIMIT_REACHED* 2)
(intern "*JOINT_POSITION_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*JOINT_POSITION_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*JOINT_POSITION_LIMIT_REACHED* 3)
(intern "*JOINT_TORQUE_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*JOINT_TORQUE_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*JOINT_TORQUE_LIMIT_REACHED* 4)
(intern "*SINGULARITY_REGION*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*SINGULARITY_REGION* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*SINGULARITY_REGION* 5)
(intern "*INVERSE_KINEMATIC_FAILED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*INVERSE_KINEMATIC_FAILED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*INVERSE_KINEMATIC_FAILED* 6)
(intern "*CARTESIAN_ACCELERATION_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*CARTESIAN_ACCELERATION_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*CARTESIAN_ACCELERATION_LIMIT_REACHED* 7)
(intern "*CARTESIAN_SPEED_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*CARTESIAN_SPEED_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*CARTESIAN_SPEED_LIMIT_REACHED* 8)
(intern "*CARTESIAN_POSITION_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*CARTESIAN_POSITION_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*CARTESIAN_POSITION_LIMIT_REACHED* 9)
(intern "*CARTESIAN_WRENCH_LIMIT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*CARTESIAN_WRENCH_LIMIT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*CARTESIAN_WRENCH_LIMIT_REACHED* 10)
(intern "*ENTERING_PROTECTION_ZONE*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*ENTERING_PROTECTION_ZONE* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*ENTERING_PROTECTION_ZONE* 11)
(intern "*WAYPOINT_REACHED*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*WAYPOINT_REACHED* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*WAYPOINT_REACHED* 12)
(intern "*TRAJECTORY_OK*" (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(shadow '*TRAJECTORY_OK* (find-package "KORTEX_DRIVER::TRAJECTORYINFOTYPE"))
(defconstant kortex_driver::TrajectoryInfoType::*TRAJECTORY_OK* 13)

(defun kortex_driver::TrajectoryInfoType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::TrajectoryInfoType::*UNSPECIFIED_TRAJECTORY_INFORMATION*)
        ((= const 1) 'kortex_driver::TrajectoryInfoType::*JOINT_ACCELERATION_LIMIT_REACHED*)
        ((= const 2) 'kortex_driver::TrajectoryInfoType::*JOINT_SPEED_LIMIT_REACHED*)
        ((= const 3) 'kortex_driver::TrajectoryInfoType::*JOINT_POSITION_LIMIT_REACHED*)
        ((= const 4) 'kortex_driver::TrajectoryInfoType::*JOINT_TORQUE_LIMIT_REACHED*)
        ((= const 5) 'kortex_driver::TrajectoryInfoType::*SINGULARITY_REGION*)
        ((= const 6) 'kortex_driver::TrajectoryInfoType::*INVERSE_KINEMATIC_FAILED*)
        ((= const 7) 'kortex_driver::TrajectoryInfoType::*CARTESIAN_ACCELERATION_LIMIT_REACHED*)
        ((= const 8) 'kortex_driver::TrajectoryInfoType::*CARTESIAN_SPEED_LIMIT_REACHED*)
        ((= const 9) 'kortex_driver::TrajectoryInfoType::*CARTESIAN_POSITION_LIMIT_REACHED*)
        ((= const 10) 'kortex_driver::TrajectoryInfoType::*CARTESIAN_WRENCH_LIMIT_REACHED*)
        ((= const 11) 'kortex_driver::TrajectoryInfoType::*ENTERING_PROTECTION_ZONE*)
        ((= const 12) 'kortex_driver::TrajectoryInfoType::*WAYPOINT_REACHED*)
        ((= const 13) 'kortex_driver::TrajectoryInfoType::*TRAJECTORY_OK*)
        (t nil)))

(defclass kortex_driver::TrajectoryInfoType
  :super ros::object
  :slots ())

(defmethod kortex_driver::TrajectoryInfoType
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

(setf (get kortex_driver::TrajectoryInfoType :md5sum-) "8bf652b45448cd88f4d8d2fc90a3634d")
(setf (get kortex_driver::TrajectoryInfoType :datatype-) "kortex_driver/TrajectoryInfoType")
(setf (get kortex_driver::TrajectoryInfoType :definition-)
      "
uint32 UNSPECIFIED_TRAJECTORY_INFORMATION = 0

uint32 JOINT_ACCELERATION_LIMIT_REACHED = 1

uint32 JOINT_SPEED_LIMIT_REACHED = 2

uint32 JOINT_POSITION_LIMIT_REACHED = 3

uint32 JOINT_TORQUE_LIMIT_REACHED = 4

uint32 SINGULARITY_REGION = 5

uint32 INVERSE_KINEMATIC_FAILED = 6

uint32 CARTESIAN_ACCELERATION_LIMIT_REACHED = 7

uint32 CARTESIAN_SPEED_LIMIT_REACHED = 8

uint32 CARTESIAN_POSITION_LIMIT_REACHED = 9

uint32 CARTESIAN_WRENCH_LIMIT_REACHED = 10

uint32 ENTERING_PROTECTION_ZONE = 11

uint32 WAYPOINT_REACHED = 12

uint32 TRAJECTORY_OK = 13

")



(provide :kortex_driver/TrajectoryInfoType "8bf652b45448cd88f4d8d2fc90a3634d")


