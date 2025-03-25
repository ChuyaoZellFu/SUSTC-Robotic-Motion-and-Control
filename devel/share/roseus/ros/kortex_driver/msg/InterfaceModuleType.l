;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterfaceModuleType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterfaceModuleType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE")
  (make-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))

(in-package "ROS")
;;//! \htmlinclude InterfaceModuleType.msg.html


(intern "*INTERFACE_MODULE_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(shadow '*INTERFACE_MODULE_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(defconstant kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_UNSPECIFIED* 0)
(intern "*INTERFACE_MODULE_TYPE_NOT_INSTALLED*" (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(shadow '*INTERFACE_MODULE_TYPE_NOT_INSTALLED* (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(defconstant kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_NOT_INSTALLED* 1)
(intern "*INTERFACE_MODULE_TYPE_L53*" (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(shadow '*INTERFACE_MODULE_TYPE_L53* (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(defconstant kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_L53* 2)
(intern "*INTERFACE_MODULE_TYPE_L31*" (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(shadow '*INTERFACE_MODULE_TYPE_L31* (find-package "KORTEX_DRIVER::INTERFACEMODULETYPE"))
(defconstant kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_L31* 3)

(defun kortex_driver::InterfaceModuleType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_NOT_INSTALLED*)
        ((= const 2) 'kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_L53*)
        ((= const 3) 'kortex_driver::InterfaceModuleType::*INTERFACE_MODULE_TYPE_L31*)
        (t nil)))

(defclass kortex_driver::InterfaceModuleType
  :super ros::object
  :slots ())

(defmethod kortex_driver::InterfaceModuleType
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

(setf (get kortex_driver::InterfaceModuleType :md5sum-) "f61dc75115f701d8cd0d9b3951030045")
(setf (get kortex_driver::InterfaceModuleType :datatype-) "kortex_driver/InterfaceModuleType")
(setf (get kortex_driver::InterfaceModuleType :definition-)
      "
uint32 INTERFACE_MODULE_TYPE_UNSPECIFIED = 0

uint32 INTERFACE_MODULE_TYPE_NOT_INSTALLED = 1

uint32 INTERFACE_MODULE_TYPE_L53 = 2

uint32 INTERFACE_MODULE_TYPE_L31 = 3

")



(provide :kortex_driver/InterfaceModuleType "f61dc75115f701d8cd0d9b3951030045")


