;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RunModes)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RunModes (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RUNMODES")
  (make-package "KORTEX_DRIVER::RUNMODES"))

(in-package "ROS")
;;//! \htmlinclude RunModes.msg.html


(intern "*RUN_MODE*" (find-package "KORTEX_DRIVER::RUNMODES"))
(shadow '*RUN_MODE* (find-package "KORTEX_DRIVER::RUNMODES"))
(defconstant kortex_driver::RunModes::*RUN_MODE* 0)
(intern "*CALIBRATION_MODE*" (find-package "KORTEX_DRIVER::RUNMODES"))
(shadow '*CALIBRATION_MODE* (find-package "KORTEX_DRIVER::RUNMODES"))
(defconstant kortex_driver::RunModes::*CALIBRATION_MODE* 1)
(intern "*CONFIGURATION_MODE*" (find-package "KORTEX_DRIVER::RUNMODES"))
(shadow '*CONFIGURATION_MODE* (find-package "KORTEX_DRIVER::RUNMODES"))
(defconstant kortex_driver::RunModes::*CONFIGURATION_MODE* 2)
(intern "*DEBUG_MODE*" (find-package "KORTEX_DRIVER::RUNMODES"))
(shadow '*DEBUG_MODE* (find-package "KORTEX_DRIVER::RUNMODES"))
(defconstant kortex_driver::RunModes::*DEBUG_MODE* 3)
(intern "*TUNING_MODE*" (find-package "KORTEX_DRIVER::RUNMODES"))
(shadow '*TUNING_MODE* (find-package "KORTEX_DRIVER::RUNMODES"))
(defconstant kortex_driver::RunModes::*TUNING_MODE* 4)

(defun kortex_driver::RunModes-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::RunModes::*RUN_MODE*)
        ((= const 1) 'kortex_driver::RunModes::*CALIBRATION_MODE*)
        ((= const 2) 'kortex_driver::RunModes::*CONFIGURATION_MODE*)
        ((= const 3) 'kortex_driver::RunModes::*DEBUG_MODE*)
        ((= const 4) 'kortex_driver::RunModes::*TUNING_MODE*)
        (t nil)))

(defclass kortex_driver::RunModes
  :super ros::object
  :slots ())

(defmethod kortex_driver::RunModes
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

(setf (get kortex_driver::RunModes :md5sum-) "4d935d7a06871eb332098b80bd9adf59")
(setf (get kortex_driver::RunModes :datatype-) "kortex_driver/RunModes")
(setf (get kortex_driver::RunModes :definition-)
      "
uint32 RUN_MODE = 0

uint32 CALIBRATION_MODE = 1

uint32 CONFIGURATION_MODE = 2

uint32 DEBUG_MODE = 3

uint32 TUNING_MODE = 4

")



(provide :kortex_driver/RunModes "4d935d7a06871eb332098b80bd9adf59")


