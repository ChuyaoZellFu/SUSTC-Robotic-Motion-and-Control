;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerInputType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerInputType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE")
  (make-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE"))

(in-package "ROS")
;;//! \htmlinclude ControllerInputType.msg.html


(intern "*UNSPECIFIED_CONTROLLER_INPUT_TYPE*" (find-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE"))
(shadow '*UNSPECIFIED_CONTROLLER_INPUT_TYPE* (find-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE"))
(defconstant kortex_driver::ControllerInputType::*UNSPECIFIED_CONTROLLER_INPUT_TYPE* 0)
(intern "*ANALOG*" (find-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE"))
(shadow '*ANALOG* (find-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE"))
(defconstant kortex_driver::ControllerInputType::*ANALOG* 1)
(intern "*DIGITAL*" (find-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE"))
(shadow '*DIGITAL* (find-package "KORTEX_DRIVER::CONTROLLERINPUTTYPE"))
(defconstant kortex_driver::ControllerInputType::*DIGITAL* 2)

(defun kortex_driver::ControllerInputType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ControllerInputType::*UNSPECIFIED_CONTROLLER_INPUT_TYPE*)
        ((= const 1) 'kortex_driver::ControllerInputType::*ANALOG*)
        ((= const 2) 'kortex_driver::ControllerInputType::*DIGITAL*)
        (t nil)))

(defclass kortex_driver::ControllerInputType
  :super ros::object
  :slots ())

(defmethod kortex_driver::ControllerInputType
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

(setf (get kortex_driver::ControllerInputType :md5sum-) "758c1628f9ac7ba8ba58d7882102ea8f")
(setf (get kortex_driver::ControllerInputType :datatype-) "kortex_driver/ControllerInputType")
(setf (get kortex_driver::ControllerInputType :definition-)
      "
uint32 UNSPECIFIED_CONTROLLER_INPUT_TYPE = 0

uint32 ANALOG = 1

uint32 DIGITAL = 2

")



(provide :kortex_driver/ControllerInputType "758c1628f9ac7ba8ba58d7882102ea8f")


