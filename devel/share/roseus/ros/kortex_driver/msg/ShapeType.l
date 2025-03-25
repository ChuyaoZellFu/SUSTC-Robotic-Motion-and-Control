;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ShapeType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ShapeType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SHAPETYPE")
  (make-package "KORTEX_DRIVER::SHAPETYPE"))

(in-package "ROS")
;;//! \htmlinclude ShapeType.msg.html


(intern "*UNSPECIFIED_SHAPE*" (find-package "KORTEX_DRIVER::SHAPETYPE"))
(shadow '*UNSPECIFIED_SHAPE* (find-package "KORTEX_DRIVER::SHAPETYPE"))
(defconstant kortex_driver::ShapeType::*UNSPECIFIED_SHAPE* 0)
(intern "*CYLINDER*" (find-package "KORTEX_DRIVER::SHAPETYPE"))
(shadow '*CYLINDER* (find-package "KORTEX_DRIVER::SHAPETYPE"))
(defconstant kortex_driver::ShapeType::*CYLINDER* 1)
(intern "*SPHERE*" (find-package "KORTEX_DRIVER::SHAPETYPE"))
(shadow '*SPHERE* (find-package "KORTEX_DRIVER::SHAPETYPE"))
(defconstant kortex_driver::ShapeType::*SPHERE* 2)
(intern "*RECTANGULAR_PRISM*" (find-package "KORTEX_DRIVER::SHAPETYPE"))
(shadow '*RECTANGULAR_PRISM* (find-package "KORTEX_DRIVER::SHAPETYPE"))
(defconstant kortex_driver::ShapeType::*RECTANGULAR_PRISM* 3)

(defun kortex_driver::ShapeType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ShapeType::*UNSPECIFIED_SHAPE*)
        ((= const 1) 'kortex_driver::ShapeType::*CYLINDER*)
        ((= const 2) 'kortex_driver::ShapeType::*SPHERE*)
        ((= const 3) 'kortex_driver::ShapeType::*RECTANGULAR_PRISM*)
        (t nil)))

(defclass kortex_driver::ShapeType
  :super ros::object
  :slots ())

(defmethod kortex_driver::ShapeType
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

(setf (get kortex_driver::ShapeType :md5sum-) "8cce3154d49982341a5a162e90cffa5a")
(setf (get kortex_driver::ShapeType :datatype-) "kortex_driver/ShapeType")
(setf (get kortex_driver::ShapeType :definition-)
      "
uint32 UNSPECIFIED_SHAPE = 0

uint32 CYLINDER = 1

uint32 SPHERE = 2

uint32 RECTANGULAR_PRISM = 3

")



(provide :kortex_driver/ShapeType "8cce3154d49982341a5a162e90cffa5a")


