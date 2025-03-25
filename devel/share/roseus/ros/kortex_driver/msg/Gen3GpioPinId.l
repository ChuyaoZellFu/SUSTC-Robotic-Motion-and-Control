;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Gen3GpioPinId)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Gen3GpioPinId (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GEN3GPIOPINID")
  (make-package "KORTEX_DRIVER::GEN3GPIOPINID"))

(in-package "ROS")
;;//! \htmlinclude Gen3GpioPinId.msg.html


(intern "*UNSPECIFIED_PIN*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*UNSPECIFIED_PIN* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*UNSPECIFIED_PIN* 0)
(intern "*GPIO_PIN_B*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_B* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_B* 1)
(intern "*GPIO_PIN_C*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_C* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_C* 2)
(intern "*GPIO_PIN_D*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_D* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_D* 3)
(intern "*GPIO_PIN_E*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_E* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_E* 4)
(intern "*GPIO_PIN_G*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_G* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_G* 5)
(intern "*GPIO_PIN_H*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_H* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_H* 6)
(intern "*GPIO_PIN_I*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_I* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_I* 7)
(intern "*GPIO_PIN_K*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_K* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_K* 8)
(intern "*GPIO_PIN_N*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_N* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_N* 9)
(intern "*GPIO_PIN_O*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_O* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_O* 10)
(intern "*GPIO_PIN_S*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_S* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_S* 11)
(intern "*GPIO_PIN_T*" (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(shadow '*GPIO_PIN_T* (find-package "KORTEX_DRIVER::GEN3GPIOPINID"))
(defconstant kortex_driver::Gen3GpioPinId::*GPIO_PIN_T* 12)

(defun kortex_driver::Gen3GpioPinId-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Gen3GpioPinId::*UNSPECIFIED_PIN*)
        ((= const 1) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_B*)
        ((= const 2) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_C*)
        ((= const 3) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_D*)
        ((= const 4) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_E*)
        ((= const 5) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_G*)
        ((= const 6) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_H*)
        ((= const 7) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_I*)
        ((= const 8) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_K*)
        ((= const 9) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_N*)
        ((= const 10) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_O*)
        ((= const 11) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_S*)
        ((= const 12) 'kortex_driver::Gen3GpioPinId::*GPIO_PIN_T*)
        (t nil)))

(defclass kortex_driver::Gen3GpioPinId
  :super ros::object
  :slots ())

(defmethod kortex_driver::Gen3GpioPinId
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

(setf (get kortex_driver::Gen3GpioPinId :md5sum-) "e4129f3451b6d28fcb9b382446ece1a9")
(setf (get kortex_driver::Gen3GpioPinId :datatype-) "kortex_driver/Gen3GpioPinId")
(setf (get kortex_driver::Gen3GpioPinId :definition-)
      "
uint32 UNSPECIFIED_PIN = 0

uint32 GPIO_PIN_B = 1

uint32 GPIO_PIN_C = 2

uint32 GPIO_PIN_D = 3

uint32 GPIO_PIN_E = 4

uint32 GPIO_PIN_G = 5

uint32 GPIO_PIN_H = 6

uint32 GPIO_PIN_I = 7

uint32 GPIO_PIN_K = 8

uint32 GPIO_PIN_N = 9

uint32 GPIO_PIN_O = 10

uint32 GPIO_PIN_S = 11

uint32 GPIO_PIN_T = 12

")



(provide :kortex_driver/Gen3GpioPinId "e4129f3451b6d28fcb9b382446ece1a9")


