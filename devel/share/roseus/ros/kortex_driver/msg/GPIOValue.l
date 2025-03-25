;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GPIOValue)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GPIOValue (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOVALUE")
  (make-package "KORTEX_DRIVER::GPIOVALUE"))

(in-package "ROS")
;;//! \htmlinclude GPIOValue.msg.html


(intern "*GPIO_VALUE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::GPIOVALUE"))
(shadow '*GPIO_VALUE_UNSPECIFIED* (find-package "KORTEX_DRIVER::GPIOVALUE"))
(defconstant kortex_driver::GPIOValue::*GPIO_VALUE_UNSPECIFIED* 0)
(intern "*GPIO_VALUE_LOW*" (find-package "KORTEX_DRIVER::GPIOVALUE"))
(shadow '*GPIO_VALUE_LOW* (find-package "KORTEX_DRIVER::GPIOVALUE"))
(defconstant kortex_driver::GPIOValue::*GPIO_VALUE_LOW* 1)
(intern "*GPIO_VALUE_HIGH*" (find-package "KORTEX_DRIVER::GPIOVALUE"))
(shadow '*GPIO_VALUE_HIGH* (find-package "KORTEX_DRIVER::GPIOVALUE"))
(defconstant kortex_driver::GPIOValue::*GPIO_VALUE_HIGH* 2)

(defun kortex_driver::GPIOValue-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GPIOValue::*GPIO_VALUE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::GPIOValue::*GPIO_VALUE_LOW*)
        ((= const 2) 'kortex_driver::GPIOValue::*GPIO_VALUE_HIGH*)
        (t nil)))

(defclass kortex_driver::GPIOValue
  :super ros::object
  :slots ())

(defmethod kortex_driver::GPIOValue
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

(setf (get kortex_driver::GPIOValue :md5sum-) "2450504e1dcf956a73a1061ef8b94f94")
(setf (get kortex_driver::GPIOValue :datatype-) "kortex_driver/GPIOValue")
(setf (get kortex_driver::GPIOValue :definition-)
      "
uint32 GPIO_VALUE_UNSPECIFIED = 0

uint32 GPIO_VALUE_LOW = 1

uint32 GPIO_VALUE_HIGH = 2

")



(provide :kortex_driver/GPIOValue "2450504e1dcf956a73a1061ef8b94f94")


