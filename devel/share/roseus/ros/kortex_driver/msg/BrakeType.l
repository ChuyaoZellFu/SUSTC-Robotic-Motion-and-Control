;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BrakeType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BrakeType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRAKETYPE")
  (make-package "KORTEX_DRIVER::BRAKETYPE"))

(in-package "ROS")
;;//! \htmlinclude BrakeType.msg.html


(intern "*BRAKE_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::BRAKETYPE"))
(shadow '*BRAKE_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::BRAKETYPE"))
(defconstant kortex_driver::BrakeType::*BRAKE_TYPE_UNSPECIFIED* 0)
(intern "*BRAKE_TYPE_NOT_INSTALLED*" (find-package "KORTEX_DRIVER::BRAKETYPE"))
(shadow '*BRAKE_TYPE_NOT_INSTALLED* (find-package "KORTEX_DRIVER::BRAKETYPE"))
(defconstant kortex_driver::BrakeType::*BRAKE_TYPE_NOT_INSTALLED* 1)
(intern "*BRAKE_TYPE_SPOKE*" (find-package "KORTEX_DRIVER::BRAKETYPE"))
(shadow '*BRAKE_TYPE_SPOKE* (find-package "KORTEX_DRIVER::BRAKETYPE"))
(defconstant kortex_driver::BrakeType::*BRAKE_TYPE_SPOKE* 2)
(intern "*BRAKE_TYPE_CLUTCH*" (find-package "KORTEX_DRIVER::BRAKETYPE"))
(shadow '*BRAKE_TYPE_CLUTCH* (find-package "KORTEX_DRIVER::BRAKETYPE"))
(defconstant kortex_driver::BrakeType::*BRAKE_TYPE_CLUTCH* 3)

(defun kortex_driver::BrakeType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::BrakeType::*BRAKE_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::BrakeType::*BRAKE_TYPE_NOT_INSTALLED*)
        ((= const 2) 'kortex_driver::BrakeType::*BRAKE_TYPE_SPOKE*)
        ((= const 3) 'kortex_driver::BrakeType::*BRAKE_TYPE_CLUTCH*)
        (t nil)))

(defclass kortex_driver::BrakeType
  :super ros::object
  :slots ())

(defmethod kortex_driver::BrakeType
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

(setf (get kortex_driver::BrakeType :md5sum-) "856901ae6854740e9adfa01cae483501")
(setf (get kortex_driver::BrakeType :datatype-) "kortex_driver/BrakeType")
(setf (get kortex_driver::BrakeType :definition-)
      "
uint32 BRAKE_TYPE_UNSPECIFIED = 0

uint32 BRAKE_TYPE_NOT_INSTALLED = 1

uint32 BRAKE_TYPE_SPOKE = 2

uint32 BRAKE_TYPE_CLUTCH = 3

")



(provide :kortex_driver/BrakeType "856901ae6854740e9adfa01cae483501")


