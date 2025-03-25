;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GpioAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GpioAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOACTION")
  (make-package "KORTEX_DRIVER::GPIOACTION"))

(in-package "ROS")
;;//! \htmlinclude GpioAction.msg.html


(intern "*UNSPECIFIED_GPIO_ACTION*" (find-package "KORTEX_DRIVER::GPIOACTION"))
(shadow '*UNSPECIFIED_GPIO_ACTION* (find-package "KORTEX_DRIVER::GPIOACTION"))
(defconstant kortex_driver::GpioAction::*UNSPECIFIED_GPIO_ACTION* 0)
(intern "*GPIOACTION_SET*" (find-package "KORTEX_DRIVER::GPIOACTION"))
(shadow '*GPIOACTION_SET* (find-package "KORTEX_DRIVER::GPIOACTION"))
(defconstant kortex_driver::GpioAction::*GPIOACTION_SET* 1)
(intern "*GPIOACTION_CLEAR*" (find-package "KORTEX_DRIVER::GPIOACTION"))
(shadow '*GPIOACTION_CLEAR* (find-package "KORTEX_DRIVER::GPIOACTION"))
(defconstant kortex_driver::GpioAction::*GPIOACTION_CLEAR* 2)
(intern "*GPIOACTION_PULSE_HIGH*" (find-package "KORTEX_DRIVER::GPIOACTION"))
(shadow '*GPIOACTION_PULSE_HIGH* (find-package "KORTEX_DRIVER::GPIOACTION"))
(defconstant kortex_driver::GpioAction::*GPIOACTION_PULSE_HIGH* 3)
(intern "*GPIOACTION_PULSE_LOW*" (find-package "KORTEX_DRIVER::GPIOACTION"))
(shadow '*GPIOACTION_PULSE_LOW* (find-package "KORTEX_DRIVER::GPIOACTION"))
(defconstant kortex_driver::GpioAction::*GPIOACTION_PULSE_LOW* 4)

(defun kortex_driver::GpioAction-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GpioAction::*UNSPECIFIED_GPIO_ACTION*)
        ((= const 1) 'kortex_driver::GpioAction::*GPIOACTION_SET*)
        ((= const 2) 'kortex_driver::GpioAction::*GPIOACTION_CLEAR*)
        ((= const 3) 'kortex_driver::GpioAction::*GPIOACTION_PULSE_HIGH*)
        ((= const 4) 'kortex_driver::GpioAction::*GPIOACTION_PULSE_LOW*)
        (t nil)))

(defclass kortex_driver::GpioAction
  :super ros::object
  :slots ())

(defmethod kortex_driver::GpioAction
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

(setf (get kortex_driver::GpioAction :md5sum-) "7271e7564b2e393d951b0684902edec6")
(setf (get kortex_driver::GpioAction :datatype-) "kortex_driver/GpioAction")
(setf (get kortex_driver::GpioAction :definition-)
      "
uint32 UNSPECIFIED_GPIO_ACTION = 0

uint32 GPIOACTION_SET = 1

uint32 GPIOACTION_CLEAR = 2

uint32 GPIOACTION_PULSE_HIGH = 3

uint32 GPIOACTION_PULSE_LOW = 4

")



(provide :kortex_driver/GpioAction "7271e7564b2e393d951b0684902edec6")


