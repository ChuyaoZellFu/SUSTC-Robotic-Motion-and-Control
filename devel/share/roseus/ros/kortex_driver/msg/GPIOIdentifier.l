;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GPIOIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GPIOIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOIDENTIFIER")
  (make-package "KORTEX_DRIVER::GPIOIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude GPIOIdentifier.msg.html


(intern "*GPIO_IDENTIFIER_UNSPECIFIED*" (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(shadow '*GPIO_IDENTIFIER_UNSPECIFIED* (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(defconstant kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_UNSPECIFIED* 0)
(intern "*GPIO_IDENTIFIER_1*" (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(shadow '*GPIO_IDENTIFIER_1* (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(defconstant kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_1* 1)
(intern "*GPIO_IDENTIFIER_2*" (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(shadow '*GPIO_IDENTIFIER_2* (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(defconstant kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_2* 2)
(intern "*GPIO_IDENTIFIER_3*" (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(shadow '*GPIO_IDENTIFIER_3* (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(defconstant kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_3* 3)
(intern "*GPIO_IDENTIFIER_4*" (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(shadow '*GPIO_IDENTIFIER_4* (find-package "KORTEX_DRIVER::GPIOIDENTIFIER"))
(defconstant kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_4* 4)

(defun kortex_driver::GPIOIdentifier-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_1*)
        ((= const 2) 'kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_2*)
        ((= const 3) 'kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_3*)
        ((= const 4) 'kortex_driver::GPIOIdentifier::*GPIO_IDENTIFIER_4*)
        (t nil)))

(defclass kortex_driver::GPIOIdentifier
  :super ros::object
  :slots ())

(defmethod kortex_driver::GPIOIdentifier
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

(setf (get kortex_driver::GPIOIdentifier :md5sum-) "6d6dda494f04fdbc115c7ef311e5d45d")
(setf (get kortex_driver::GPIOIdentifier :datatype-) "kortex_driver/GPIOIdentifier")
(setf (get kortex_driver::GPIOIdentifier :definition-)
      "
uint32 GPIO_IDENTIFIER_UNSPECIFIED = 0

uint32 GPIO_IDENTIFIER_1 = 1

uint32 GPIO_IDENTIFIER_2 = 2

uint32 GPIO_IDENTIFIER_3 = 3

uint32 GPIO_IDENTIFIER_4 = 4

")



(provide :kortex_driver/GPIOIdentifier "6d6dda494f04fdbc115c7ef311e5d45d")


