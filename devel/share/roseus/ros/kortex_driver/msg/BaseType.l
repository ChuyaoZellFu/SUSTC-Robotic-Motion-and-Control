;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BaseType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BaseType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASETYPE")
  (make-package "KORTEX_DRIVER::BASETYPE"))

(in-package "ROS")
;;//! \htmlinclude BaseType.msg.html


(intern "*BASE_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::BASETYPE"))
(shadow '*BASE_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::BASETYPE"))
(defconstant kortex_driver::BaseType::*BASE_TYPE_UNSPECIFIED* 0)
(intern "*BASE_TYPE_L53_QUICK_CONNECT*" (find-package "KORTEX_DRIVER::BASETYPE"))
(shadow '*BASE_TYPE_L53_QUICK_CONNECT* (find-package "KORTEX_DRIVER::BASETYPE"))
(defconstant kortex_driver::BaseType::*BASE_TYPE_L53_QUICK_CONNECT* 1)
(intern "*BASE_TYPE_L53_FIXED*" (find-package "KORTEX_DRIVER::BASETYPE"))
(shadow '*BASE_TYPE_L53_FIXED* (find-package "KORTEX_DRIVER::BASETYPE"))
(defconstant kortex_driver::BaseType::*BASE_TYPE_L53_FIXED* 2)
(intern "*BASE_TYPE_L31_FIXED*" (find-package "KORTEX_DRIVER::BASETYPE"))
(shadow '*BASE_TYPE_L31_FIXED* (find-package "KORTEX_DRIVER::BASETYPE"))
(defconstant kortex_driver::BaseType::*BASE_TYPE_L31_FIXED* 3)
(intern "*BASE_TYPE_HDK_FIXED*" (find-package "KORTEX_DRIVER::BASETYPE"))
(shadow '*BASE_TYPE_HDK_FIXED* (find-package "KORTEX_DRIVER::BASETYPE"))
(defconstant kortex_driver::BaseType::*BASE_TYPE_HDK_FIXED* 4)

(defun kortex_driver::BaseType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::BaseType::*BASE_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::BaseType::*BASE_TYPE_L53_QUICK_CONNECT*)
        ((= const 2) 'kortex_driver::BaseType::*BASE_TYPE_L53_FIXED*)
        ((= const 3) 'kortex_driver::BaseType::*BASE_TYPE_L31_FIXED*)
        ((= const 4) 'kortex_driver::BaseType::*BASE_TYPE_HDK_FIXED*)
        (t nil)))

(defclass kortex_driver::BaseType
  :super ros::object
  :slots ())

(defmethod kortex_driver::BaseType
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

(setf (get kortex_driver::BaseType :md5sum-) "7e0a5914d628ae4f54d3bdfb80a2c377")
(setf (get kortex_driver::BaseType :datatype-) "kortex_driver/BaseType")
(setf (get kortex_driver::BaseType :definition-)
      "
uint32 BASE_TYPE_UNSPECIFIED = 0

uint32 BASE_TYPE_L53_QUICK_CONNECT = 1

uint32 BASE_TYPE_L53_FIXED = 2

uint32 BASE_TYPE_L31_FIXED = 3

uint32 BASE_TYPE_HDK_FIXED = 4

")



(provide :kortex_driver/BaseType "7e0a5914d628ae4f54d3bdfb80a2c377")


