;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::VisionModuleType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'VisionModuleType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::VISIONMODULETYPE")
  (make-package "KORTEX_DRIVER::VISIONMODULETYPE"))

(in-package "ROS")
;;//! \htmlinclude VisionModuleType.msg.html


(intern "*VISION_MODULE_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(shadow '*VISION_MODULE_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(defconstant kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_UNSPECIFIED* 0)
(intern "*VISION_MODULE_TYPE_NOT_INSTALLED*" (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(shadow '*VISION_MODULE_TYPE_NOT_INSTALLED* (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(defconstant kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_NOT_INSTALLED* 1)
(intern "*VISION_MODULE_TYPE_L53_2D3D*" (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(shadow '*VISION_MODULE_TYPE_L53_2D3D* (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(defconstant kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_L53_2D3D* 2)
(intern "*VISION_MODULE_TYPE_L53_2D*" (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(shadow '*VISION_MODULE_TYPE_L53_2D* (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(defconstant kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_L53_2D* 3)
(intern "*VISION_MODULE_TYPE_EOD*" (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(shadow '*VISION_MODULE_TYPE_EOD* (find-package "KORTEX_DRIVER::VISIONMODULETYPE"))
(defconstant kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_EOD* 4)

(defun kortex_driver::VisionModuleType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_NOT_INSTALLED*)
        ((= const 2) 'kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_L53_2D3D*)
        ((= const 3) 'kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_L53_2D*)
        ((= const 4) 'kortex_driver::VisionModuleType::*VISION_MODULE_TYPE_EOD*)
        (t nil)))

(defclass kortex_driver::VisionModuleType
  :super ros::object
  :slots ())

(defmethod kortex_driver::VisionModuleType
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

(setf (get kortex_driver::VisionModuleType :md5sum-) "bb31bed96ad6e07f3b30f11301afe3f6")
(setf (get kortex_driver::VisionModuleType :datatype-) "kortex_driver/VisionModuleType")
(setf (get kortex_driver::VisionModuleType :definition-)
      "
uint32 VISION_MODULE_TYPE_UNSPECIFIED = 0

uint32 VISION_MODULE_TYPE_NOT_INSTALLED = 1

uint32 VISION_MODULE_TYPE_L53_2D3D = 2

uint32 VISION_MODULE_TYPE_L53_2D = 3

uint32 VISION_MODULE_TYPE_EOD = 4

")



(provide :kortex_driver/VisionModuleType "bb31bed96ad6e07f3b30f11301afe3f6")


