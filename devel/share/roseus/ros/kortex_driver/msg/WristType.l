;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WristType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WristType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WRISTTYPE")
  (make-package "KORTEX_DRIVER::WRISTTYPE"))

(in-package "ROS")
;;//! \htmlinclude WristType.msg.html


(intern "*WRIST_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::WRISTTYPE"))
(shadow '*WRIST_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::WRISTTYPE"))
(defconstant kortex_driver::WristType::*WRIST_TYPE_UNSPECIFIED* 0)
(intern "*WRIST_TYPE_NOT_APPLICABLE*" (find-package "KORTEX_DRIVER::WRISTTYPE"))
(shadow '*WRIST_TYPE_NOT_APPLICABLE* (find-package "KORTEX_DRIVER::WRISTTYPE"))
(defconstant kortex_driver::WristType::*WRIST_TYPE_NOT_APPLICABLE* 1)
(intern "*WRIST_TYPE_SPHERICAL*" (find-package "KORTEX_DRIVER::WRISTTYPE"))
(shadow '*WRIST_TYPE_SPHERICAL* (find-package "KORTEX_DRIVER::WRISTTYPE"))
(defconstant kortex_driver::WristType::*WRIST_TYPE_SPHERICAL* 2)
(intern "*WRIST_TYPE_CURVED*" (find-package "KORTEX_DRIVER::WRISTTYPE"))
(shadow '*WRIST_TYPE_CURVED* (find-package "KORTEX_DRIVER::WRISTTYPE"))
(defconstant kortex_driver::WristType::*WRIST_TYPE_CURVED* 3)

(defun kortex_driver::WristType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::WristType::*WRIST_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::WristType::*WRIST_TYPE_NOT_APPLICABLE*)
        ((= const 2) 'kortex_driver::WristType::*WRIST_TYPE_SPHERICAL*)
        ((= const 3) 'kortex_driver::WristType::*WRIST_TYPE_CURVED*)
        (t nil)))

(defclass kortex_driver::WristType
  :super ros::object
  :slots ())

(defmethod kortex_driver::WristType
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

(setf (get kortex_driver::WristType :md5sum-) "dce76d4e649d6e568f93c37bf10d6238")
(setf (get kortex_driver::WristType :datatype-) "kortex_driver/WristType")
(setf (get kortex_driver::WristType :definition-)
      "
uint32 WRIST_TYPE_UNSPECIFIED = 0

uint32 WRIST_TYPE_NOT_APPLICABLE = 1

uint32 WRIST_TYPE_SPHERICAL = 2

uint32 WRIST_TYPE_CURVED = 3

")



(provide :kortex_driver/WristType "dce76d4e649d6e568f93c37bf10d6238")


