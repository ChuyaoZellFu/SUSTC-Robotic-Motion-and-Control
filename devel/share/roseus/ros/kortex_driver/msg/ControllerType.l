;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERTYPE")
  (make-package "KORTEX_DRIVER::CONTROLLERTYPE"))

(in-package "ROS")
;;//! \htmlinclude ControllerType.msg.html


(intern "*UNSPECIFIED_CONTROLLER_TYPE*" (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(shadow '*UNSPECIFIED_CONTROLLER_TYPE* (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(defconstant kortex_driver::ControllerType::*UNSPECIFIED_CONTROLLER_TYPE* 0)
(intern "*XBOX_CONTROLLER*" (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(shadow '*XBOX_CONTROLLER* (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(defconstant kortex_driver::ControllerType::*XBOX_CONTROLLER* 1)
(intern "*WRIST_CONTROLLER*" (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(shadow '*WRIST_CONTROLLER* (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(defconstant kortex_driver::ControllerType::*WRIST_CONTROLLER* 2)
(intern "*BASIC_JOYSTICK_CONTROLLER*" (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(shadow '*BASIC_JOYSTICK_CONTROLLER* (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(defconstant kortex_driver::ControllerType::*BASIC_JOYSTICK_CONTROLLER* 3)
(intern "*BASE_GPIO_CONTROLLER*" (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(shadow '*BASE_GPIO_CONTROLLER* (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(defconstant kortex_driver::ControllerType::*BASE_GPIO_CONTROLLER* 4)
(intern "*GPIO_JOYSTICK*" (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(shadow '*GPIO_JOYSTICK* (find-package "KORTEX_DRIVER::CONTROLLERTYPE"))
(defconstant kortex_driver::ControllerType::*GPIO_JOYSTICK* 5)

(defun kortex_driver::ControllerType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ControllerType::*UNSPECIFIED_CONTROLLER_TYPE*)
        ((= const 1) 'kortex_driver::ControllerType::*XBOX_CONTROLLER*)
        ((= const 2) 'kortex_driver::ControllerType::*WRIST_CONTROLLER*)
        ((= const 3) 'kortex_driver::ControllerType::*BASIC_JOYSTICK_CONTROLLER*)
        ((= const 4) 'kortex_driver::ControllerType::*BASE_GPIO_CONTROLLER*)
        ((= const 5) 'kortex_driver::ControllerType::*GPIO_JOYSTICK*)
        (t nil)))

(defclass kortex_driver::ControllerType
  :super ros::object
  :slots ())

(defmethod kortex_driver::ControllerType
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

(setf (get kortex_driver::ControllerType :md5sum-) "482367554f194246d15dc5fb1ede90ac")
(setf (get kortex_driver::ControllerType :datatype-) "kortex_driver/ControllerType")
(setf (get kortex_driver::ControllerType :definition-)
      "
uint32 UNSPECIFIED_CONTROLLER_TYPE = 0

uint32 XBOX_CONTROLLER = 1

uint32 WRIST_CONTROLLER = 2

uint32 BASIC_JOYSTICK_CONTROLLER = 3

uint32 BASE_GPIO_CONTROLLER = 4

uint32 GPIO_JOYSTICK = 5

")



(provide :kortex_driver/ControllerType "482367554f194246d15dc5fb1ede90ac")


