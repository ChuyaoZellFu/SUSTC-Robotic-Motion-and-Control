;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GPIOMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GPIOMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOMODE")
  (make-package "KORTEX_DRIVER::GPIOMODE"))

(in-package "ROS")
;;//! \htmlinclude GPIOMode.msg.html


(intern "*GPIO_MODE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::GPIOMODE"))
(shadow '*GPIO_MODE_UNSPECIFIED* (find-package "KORTEX_DRIVER::GPIOMODE"))
(defconstant kortex_driver::GPIOMode::*GPIO_MODE_UNSPECIFIED* 0)
(intern "*GPIO_MODE_INPUT_FLOATING*" (find-package "KORTEX_DRIVER::GPIOMODE"))
(shadow '*GPIO_MODE_INPUT_FLOATING* (find-package "KORTEX_DRIVER::GPIOMODE"))
(defconstant kortex_driver::GPIOMode::*GPIO_MODE_INPUT_FLOATING* 1)
(intern "*GPIO_MODE_OUTPUT_PUSH_PULL*" (find-package "KORTEX_DRIVER::GPIOMODE"))
(shadow '*GPIO_MODE_OUTPUT_PUSH_PULL* (find-package "KORTEX_DRIVER::GPIOMODE"))
(defconstant kortex_driver::GPIOMode::*GPIO_MODE_OUTPUT_PUSH_PULL* 2)
(intern "*GPIO_MODE_OUTPUT_OPEN_DRAIN*" (find-package "KORTEX_DRIVER::GPIOMODE"))
(shadow '*GPIO_MODE_OUTPUT_OPEN_DRAIN* (find-package "KORTEX_DRIVER::GPIOMODE"))
(defconstant kortex_driver::GPIOMode::*GPIO_MODE_OUTPUT_OPEN_DRAIN* 3)

(defun kortex_driver::GPIOMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GPIOMode::*GPIO_MODE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::GPIOMode::*GPIO_MODE_INPUT_FLOATING*)
        ((= const 2) 'kortex_driver::GPIOMode::*GPIO_MODE_OUTPUT_PUSH_PULL*)
        ((= const 3) 'kortex_driver::GPIOMode::*GPIO_MODE_OUTPUT_OPEN_DRAIN*)
        (t nil)))

(defclass kortex_driver::GPIOMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::GPIOMode
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

(setf (get kortex_driver::GPIOMode :md5sum-) "67dcb56ddb7c7d3c3c961e401c8325a4")
(setf (get kortex_driver::GPIOMode :datatype-) "kortex_driver/GPIOMode")
(setf (get kortex_driver::GPIOMode :definition-)
      "
uint32 GPIO_MODE_UNSPECIFIED = 0

uint32 GPIO_MODE_INPUT_FLOATING = 1

uint32 GPIO_MODE_OUTPUT_PUSH_PULL = 2

uint32 GPIO_MODE_OUTPUT_OPEN_DRAIN = 3

")



(provide :kortex_driver/GPIOMode "67dcb56ddb7c7d3c3c961e401c8325a4")


