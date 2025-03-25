;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerBehavior)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerBehavior (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR")
  (make-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))

(in-package "ROS")
;;//! \htmlinclude ControllerBehavior.msg.html


(intern "*UNSPECIFIED_CONTROLLER_BEHAVIOR*" (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(shadow '*UNSPECIFIED_CONTROLLER_BEHAVIOR* (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(defconstant kortex_driver::ControllerBehavior::*UNSPECIFIED_CONTROLLER_BEHAVIOR* 0)
(intern "*CONTROLLER_BUTTON_DOWN*" (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(shadow '*CONTROLLER_BUTTON_DOWN* (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(defconstant kortex_driver::ControllerBehavior::*CONTROLLER_BUTTON_DOWN* 1)
(intern "*CONTROLLER_BUTTON_UP*" (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(shadow '*CONTROLLER_BUTTON_UP* (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(defconstant kortex_driver::ControllerBehavior::*CONTROLLER_BUTTON_UP* 2)
(intern "*CONTROLLER_AXIS_POSITIVE*" (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(shadow '*CONTROLLER_AXIS_POSITIVE* (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(defconstant kortex_driver::ControllerBehavior::*CONTROLLER_AXIS_POSITIVE* 3)
(intern "*CONTROLLER_AXIS_NEGATIVE*" (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(shadow '*CONTROLLER_AXIS_NEGATIVE* (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(defconstant kortex_driver::ControllerBehavior::*CONTROLLER_AXIS_NEGATIVE* 4)
(intern "*CONTROLLER_BUTTON_CLICK*" (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(shadow '*CONTROLLER_BUTTON_CLICK* (find-package "KORTEX_DRIVER::CONTROLLERBEHAVIOR"))
(defconstant kortex_driver::ControllerBehavior::*CONTROLLER_BUTTON_CLICK* 5)

(defun kortex_driver::ControllerBehavior-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ControllerBehavior::*UNSPECIFIED_CONTROLLER_BEHAVIOR*)
        ((= const 1) 'kortex_driver::ControllerBehavior::*CONTROLLER_BUTTON_DOWN*)
        ((= const 2) 'kortex_driver::ControllerBehavior::*CONTROLLER_BUTTON_UP*)
        ((= const 3) 'kortex_driver::ControllerBehavior::*CONTROLLER_AXIS_POSITIVE*)
        ((= const 4) 'kortex_driver::ControllerBehavior::*CONTROLLER_AXIS_NEGATIVE*)
        ((= const 5) 'kortex_driver::ControllerBehavior::*CONTROLLER_BUTTON_CLICK*)
        (t nil)))

(defclass kortex_driver::ControllerBehavior
  :super ros::object
  :slots ())

(defmethod kortex_driver::ControllerBehavior
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

(setf (get kortex_driver::ControllerBehavior :md5sum-) "12553caa8500dc92a4c4cfda3f89fc6d")
(setf (get kortex_driver::ControllerBehavior :datatype-) "kortex_driver/ControllerBehavior")
(setf (get kortex_driver::ControllerBehavior :definition-)
      "
uint32 UNSPECIFIED_CONTROLLER_BEHAVIOR = 0

uint32 CONTROLLER_BUTTON_DOWN = 1

uint32 CONTROLLER_BUTTON_UP = 2

uint32 CONTROLLER_AXIS_POSITIVE = 3

uint32 CONTROLLER_AXIS_NEGATIVE = 4

uint32 CONTROLLER_BUTTON_CLICK = 5

")



(provide :kortex_driver/ControllerBehavior "12553caa8500dc92a4c4cfda3f89fc6d")


