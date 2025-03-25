;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_ControlMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_ControlMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_CONTROLMODE")
  (make-package "KORTEX_DRIVER::BASE_CONTROLMODE"))

(in-package "ROS")
;;//! \htmlinclude Base_ControlMode.msg.html


(intern "*UNSPECIFIED_CONTROL_MODE*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*UNSPECIFIED_CONTROL_MODE* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*UNSPECIFIED_CONTROL_MODE* 0)
(intern "*ANGULAR_JOYSTICK*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*ANGULAR_JOYSTICK* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*ANGULAR_JOYSTICK* 1)
(intern "*CARTESIAN_JOYSTICK*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*CARTESIAN_JOYSTICK* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*CARTESIAN_JOYSTICK* 2)
(intern "*ANGULAR_TRAJECTORY*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*ANGULAR_TRAJECTORY* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*ANGULAR_TRAJECTORY* 4)
(intern "*CARTESIAN_TRAJECTORY*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*CARTESIAN_TRAJECTORY* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*CARTESIAN_TRAJECTORY* 5)
(intern "*CARTESIAN_ADMITTANCE*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*CARTESIAN_ADMITTANCE* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*CARTESIAN_ADMITTANCE* 6)
(intern "*JOINT_ADMITTANCE*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*JOINT_ADMITTANCE* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*JOINT_ADMITTANCE* 7)
(intern "*NULL_SPACE_ADMITTANCE*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*NULL_SPACE_ADMITTANCE* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*NULL_SPACE_ADMITTANCE* 8)
(intern "*FORCE_CONTROL*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*FORCE_CONTROL* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*FORCE_CONTROL* 10)
(intern "*FORCE_CONTROL_MOTION_RESTRICTED*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*FORCE_CONTROL_MOTION_RESTRICTED* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*FORCE_CONTROL_MOTION_RESTRICTED* 11)
(intern "*IDLE*" (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(shadow '*IDLE* (find-package "KORTEX_DRIVER::BASE_CONTROLMODE"))
(defconstant kortex_driver::Base_ControlMode::*IDLE* 13)

(defun kortex_driver::Base_ControlMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Base_ControlMode::*UNSPECIFIED_CONTROL_MODE*)
        ((= const 1) 'kortex_driver::Base_ControlMode::*ANGULAR_JOYSTICK*)
        ((= const 2) 'kortex_driver::Base_ControlMode::*CARTESIAN_JOYSTICK*)
        ((= const 4) 'kortex_driver::Base_ControlMode::*ANGULAR_TRAJECTORY*)
        ((= const 5) 'kortex_driver::Base_ControlMode::*CARTESIAN_TRAJECTORY*)
        ((= const 6) 'kortex_driver::Base_ControlMode::*CARTESIAN_ADMITTANCE*)
        ((= const 7) 'kortex_driver::Base_ControlMode::*JOINT_ADMITTANCE*)
        ((= const 8) 'kortex_driver::Base_ControlMode::*NULL_SPACE_ADMITTANCE*)
        ((= const 10) 'kortex_driver::Base_ControlMode::*FORCE_CONTROL*)
        ((= const 11) 'kortex_driver::Base_ControlMode::*FORCE_CONTROL_MOTION_RESTRICTED*)
        ((= const 13) 'kortex_driver::Base_ControlMode::*IDLE*)
        (t nil)))

(defclass kortex_driver::Base_ControlMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::Base_ControlMode
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

(setf (get kortex_driver::Base_ControlMode :md5sum-) "027ce394a2c0dcd8cf27153deda7b144")
(setf (get kortex_driver::Base_ControlMode :datatype-) "kortex_driver/Base_ControlMode")
(setf (get kortex_driver::Base_ControlMode :definition-)
      "
uint32 UNSPECIFIED_CONTROL_MODE = 0

uint32 ANGULAR_JOYSTICK = 1

uint32 CARTESIAN_JOYSTICK = 2

uint32 ANGULAR_TRAJECTORY = 4

uint32 CARTESIAN_TRAJECTORY = 5

uint32 CARTESIAN_ADMITTANCE = 6

uint32 JOINT_ADMITTANCE = 7

uint32 NULL_SPACE_ADMITTANCE = 8

uint32 FORCE_CONTROL = 10

uint32 FORCE_CONTROL_MOTION_RESTRICTED = 11

uint32 IDLE = 13

")



(provide :kortex_driver/Base_ControlMode "027ce394a2c0dcd8cf27153deda7b144")


