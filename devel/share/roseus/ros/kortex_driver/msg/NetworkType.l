;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::NetworkType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'NetworkType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::NETWORKTYPE")
  (make-package "KORTEX_DRIVER::NETWORKTYPE"))

(in-package "ROS")
;;//! \htmlinclude NetworkType.msg.html


(intern "*UNSPECIFIED_NETWORK_TYPE*" (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(shadow '*UNSPECIFIED_NETWORK_TYPE* (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(defconstant kortex_driver::NetworkType::*UNSPECIFIED_NETWORK_TYPE* 0)
(intern "*WIFI*" (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(shadow '*WIFI* (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(defconstant kortex_driver::NetworkType::*WIFI* 1)
(intern "*WIRED_ETHERNET*" (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(shadow '*WIRED_ETHERNET* (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(defconstant kortex_driver::NetworkType::*WIRED_ETHERNET* 2)
(intern "*WIRED_MICROUSB*" (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(shadow '*WIRED_MICROUSB* (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(defconstant kortex_driver::NetworkType::*WIRED_MICROUSB* 3)
(intern "*WIRED_USB*" (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(shadow '*WIRED_USB* (find-package "KORTEX_DRIVER::NETWORKTYPE"))
(defconstant kortex_driver::NetworkType::*WIRED_USB* 4)

(defun kortex_driver::NetworkType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::NetworkType::*UNSPECIFIED_NETWORK_TYPE*)
        ((= const 1) 'kortex_driver::NetworkType::*WIFI*)
        ((= const 2) 'kortex_driver::NetworkType::*WIRED_ETHERNET*)
        ((= const 3) 'kortex_driver::NetworkType::*WIRED_MICROUSB*)
        ((= const 4) 'kortex_driver::NetworkType::*WIRED_USB*)
        (t nil)))

(defclass kortex_driver::NetworkType
  :super ros::object
  :slots ())

(defmethod kortex_driver::NetworkType
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

(setf (get kortex_driver::NetworkType :md5sum-) "de77e3c574da819850bde657b3223236")
(setf (get kortex_driver::NetworkType :datatype-) "kortex_driver/NetworkType")
(setf (get kortex_driver::NetworkType :definition-)
      "
uint32 UNSPECIFIED_NETWORK_TYPE = 0

uint32 WIFI = 1

uint32 WIRED_ETHERNET = 2

uint32 WIRED_MICROUSB = 3

uint32 WIRED_USB = 4

")



(provide :kortex_driver/NetworkType "de77e3c574da819850bde657b3223236")


