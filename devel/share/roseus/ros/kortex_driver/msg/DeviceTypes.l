;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeviceTypes)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeviceTypes (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DEVICETYPES")
  (make-package "KORTEX_DRIVER::DEVICETYPES"))

(in-package "ROS")
;;//! \htmlinclude DeviceTypes.msg.html


(intern "*UNSPECIFIED_DEVICE_TYPE*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*UNSPECIFIED_DEVICE_TYPE* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*UNSPECIFIED_DEVICE_TYPE* 0)
(intern "*BASE*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*BASE* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*BASE* 1)
(intern "*VISION*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*VISION* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*VISION* 2)
(intern "*BIG_ACTUATOR*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*BIG_ACTUATOR* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*BIG_ACTUATOR* 3)
(intern "*SMALL_ACTUATOR*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*SMALL_ACTUATOR* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*SMALL_ACTUATOR* 4)
(intern "*INTERCONNECT*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*INTERCONNECT* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*INTERCONNECT* 5)
(intern "*GRIPPER*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*GRIPPER* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*GRIPPER* 6)
(intern "*MEDIUM_ACTUATOR*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*MEDIUM_ACTUATOR* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*MEDIUM_ACTUATOR* 7)
(intern "*XBIG_ACTUATOR*" (find-package "KORTEX_DRIVER::DEVICETYPES"))
(shadow '*XBIG_ACTUATOR* (find-package "KORTEX_DRIVER::DEVICETYPES"))
(defconstant kortex_driver::DeviceTypes::*XBIG_ACTUATOR* 8)

(defun kortex_driver::DeviceTypes-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::DeviceTypes::*UNSPECIFIED_DEVICE_TYPE*)
        ((= const 1) 'kortex_driver::DeviceTypes::*BASE*)
        ((= const 2) 'kortex_driver::DeviceTypes::*VISION*)
        ((= const 3) 'kortex_driver::DeviceTypes::*BIG_ACTUATOR*)
        ((= const 4) 'kortex_driver::DeviceTypes::*SMALL_ACTUATOR*)
        ((= const 5) 'kortex_driver::DeviceTypes::*INTERCONNECT*)
        ((= const 6) 'kortex_driver::DeviceTypes::*GRIPPER*)
        ((= const 7) 'kortex_driver::DeviceTypes::*MEDIUM_ACTUATOR*)
        ((= const 8) 'kortex_driver::DeviceTypes::*XBIG_ACTUATOR*)
        (t nil)))

(defclass kortex_driver::DeviceTypes
  :super ros::object
  :slots ())

(defmethod kortex_driver::DeviceTypes
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

(setf (get kortex_driver::DeviceTypes :md5sum-) "46c218f31055194b4054a2785d94d7f0")
(setf (get kortex_driver::DeviceTypes :datatype-) "kortex_driver/DeviceTypes")
(setf (get kortex_driver::DeviceTypes :definition-)
      "
uint32 UNSPECIFIED_DEVICE_TYPE = 0

uint32 BASE = 1

uint32 VISION = 2

uint32 BIG_ACTUATOR = 3

uint32 SMALL_ACTUATOR = 4

uint32 INTERCONNECT = 5

uint32 GRIPPER = 6

uint32 MEDIUM_ACTUATOR = 7

uint32 XBIG_ACTUATOR = 8

")



(provide :kortex_driver/DeviceTypes "46c218f31055194b4054a2785d94d7f0")


