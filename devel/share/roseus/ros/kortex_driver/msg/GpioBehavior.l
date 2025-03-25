;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GpioBehavior)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GpioBehavior (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOBEHAVIOR")
  (make-package "KORTEX_DRIVER::GPIOBEHAVIOR"))

(in-package "ROS")
;;//! \htmlinclude GpioBehavior.msg.html


(intern "*UNSPECIFIED_GPIO_BEHAVIOR*" (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(shadow '*UNSPECIFIED_GPIO_BEHAVIOR* (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(defconstant kortex_driver::GpioBehavior::*UNSPECIFIED_GPIO_BEHAVIOR* 0)
(intern "*GPIO_FALLING*" (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(shadow '*GPIO_FALLING* (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(defconstant kortex_driver::GpioBehavior::*GPIO_FALLING* 1)
(intern "*GPIO_RISING*" (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(shadow '*GPIO_RISING* (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(defconstant kortex_driver::GpioBehavior::*GPIO_RISING* 2)
(intern "*GPIO_PULSE_LOW*" (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(shadow '*GPIO_PULSE_LOW* (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(defconstant kortex_driver::GpioBehavior::*GPIO_PULSE_LOW* 3)
(intern "*GPIO_PULSE_HIGH*" (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(shadow '*GPIO_PULSE_HIGH* (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(defconstant kortex_driver::GpioBehavior::*GPIO_PULSE_HIGH* 4)
(intern "*GPIO_ANALOG_CHANGE*" (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(shadow '*GPIO_ANALOG_CHANGE* (find-package "KORTEX_DRIVER::GPIOBEHAVIOR"))
(defconstant kortex_driver::GpioBehavior::*GPIO_ANALOG_CHANGE* 5)

(defun kortex_driver::GpioBehavior-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GpioBehavior::*UNSPECIFIED_GPIO_BEHAVIOR*)
        ((= const 1) 'kortex_driver::GpioBehavior::*GPIO_FALLING*)
        ((= const 2) 'kortex_driver::GpioBehavior::*GPIO_RISING*)
        ((= const 3) 'kortex_driver::GpioBehavior::*GPIO_PULSE_LOW*)
        ((= const 4) 'kortex_driver::GpioBehavior::*GPIO_PULSE_HIGH*)
        ((= const 5) 'kortex_driver::GpioBehavior::*GPIO_ANALOG_CHANGE*)
        (t nil)))

(defclass kortex_driver::GpioBehavior
  :super ros::object
  :slots ())

(defmethod kortex_driver::GpioBehavior
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

(setf (get kortex_driver::GpioBehavior :md5sum-) "a285ed144abb1d2ba02c8fa81ee67296")
(setf (get kortex_driver::GpioBehavior :datatype-) "kortex_driver/GpioBehavior")
(setf (get kortex_driver::GpioBehavior :definition-)
      "
uint32 UNSPECIFIED_GPIO_BEHAVIOR = 0

uint32 GPIO_FALLING = 1

uint32 GPIO_RISING = 2

uint32 GPIO_PULSE_LOW = 3

uint32 GPIO_PULSE_HIGH = 4

uint32 GPIO_ANALOG_CHANGE = 5

")



(provide :kortex_driver/GpioBehavior "a285ed144abb1d2ba02c8fa81ee67296")


