;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorConfig_ControlMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorConfig_ControlMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))

(in-package "ROS")
;;//! \htmlinclude ActuatorConfig_ControlMode.msg.html


(intern "*NONE*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(shadow '*NONE* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(defconstant kortex_driver::ActuatorConfig_ControlMode::*NONE* 0)
(intern "*POSITION*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(shadow '*POSITION* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(defconstant kortex_driver::ActuatorConfig_ControlMode::*POSITION* 1)
(intern "*VELOCITY*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(shadow '*VELOCITY* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(defconstant kortex_driver::ActuatorConfig_ControlMode::*VELOCITY* 2)
(intern "*TORQUE*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(shadow '*TORQUE* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(defconstant kortex_driver::ActuatorConfig_ControlMode::*TORQUE* 3)
(intern "*CURRENT*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(shadow '*CURRENT* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(defconstant kortex_driver::ActuatorConfig_ControlMode::*CURRENT* 4)
(intern "*CUSTOM*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(shadow '*CUSTOM* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(defconstant kortex_driver::ActuatorConfig_ControlMode::*CUSTOM* 5)
(intern "*TORQUE_HIGH_VELOCITY*" (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(shadow '*TORQUE_HIGH_VELOCITY* (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODE"))
(defconstant kortex_driver::ActuatorConfig_ControlMode::*TORQUE_HIGH_VELOCITY* 6)

(defun kortex_driver::ActuatorConfig_ControlMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ActuatorConfig_ControlMode::*NONE*)
        ((= const 1) 'kortex_driver::ActuatorConfig_ControlMode::*POSITION*)
        ((= const 2) 'kortex_driver::ActuatorConfig_ControlMode::*VELOCITY*)
        ((= const 3) 'kortex_driver::ActuatorConfig_ControlMode::*TORQUE*)
        ((= const 4) 'kortex_driver::ActuatorConfig_ControlMode::*CURRENT*)
        ((= const 5) 'kortex_driver::ActuatorConfig_ControlMode::*CUSTOM*)
        ((= const 6) 'kortex_driver::ActuatorConfig_ControlMode::*TORQUE_HIGH_VELOCITY*)
        (t nil)))

(defclass kortex_driver::ActuatorConfig_ControlMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::ActuatorConfig_ControlMode
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

(setf (get kortex_driver::ActuatorConfig_ControlMode :md5sum-) "bb12ce17d437184d7baad58c2fe8b556")
(setf (get kortex_driver::ActuatorConfig_ControlMode :datatype-) "kortex_driver/ActuatorConfig_ControlMode")
(setf (get kortex_driver::ActuatorConfig_ControlMode :definition-)
      "
uint32 NONE = 0

uint32 POSITION = 1

uint32 VELOCITY = 2

uint32 TORQUE = 3

uint32 CURRENT = 4

uint32 CUSTOM = 5

uint32 TORQUE_HIGH_VELOCITY = 6

")



(provide :kortex_driver/ActuatorConfig_ControlMode "bb12ce17d437184d7baad58c2fe8b556")


