;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WifiSecurityType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WifiSecurityType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WIFISECURITYTYPE")
  (make-package "KORTEX_DRIVER::WIFISECURITYTYPE"))

(in-package "ROS")
;;//! \htmlinclude WifiSecurityType.msg.html


(intern "*UNSPECIFIED_AUTHENTICATION*" (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(shadow '*UNSPECIFIED_AUTHENTICATION* (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(defconstant kortex_driver::WifiSecurityType::*UNSPECIFIED_AUTHENTICATION* 0)
(intern "*WEP*" (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(shadow '*WEP* (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(defconstant kortex_driver::WifiSecurityType::*WEP* 1)
(intern "*WPA2_PERSONAL*" (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(shadow '*WPA2_PERSONAL* (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(defconstant kortex_driver::WifiSecurityType::*WPA2_PERSONAL* 2)
(intern "*WPA_PERSONAL*" (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(shadow '*WPA_PERSONAL* (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(defconstant kortex_driver::WifiSecurityType::*WPA_PERSONAL* 4)
(intern "*NO_AUTHENTICATION*" (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(shadow '*NO_AUTHENTICATION* (find-package "KORTEX_DRIVER::WIFISECURITYTYPE"))
(defconstant kortex_driver::WifiSecurityType::*NO_AUTHENTICATION* 8)

(defun kortex_driver::WifiSecurityType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::WifiSecurityType::*UNSPECIFIED_AUTHENTICATION*)
        ((= const 1) 'kortex_driver::WifiSecurityType::*WEP*)
        ((= const 2) 'kortex_driver::WifiSecurityType::*WPA2_PERSONAL*)
        ((= const 4) 'kortex_driver::WifiSecurityType::*WPA_PERSONAL*)
        ((= const 8) 'kortex_driver::WifiSecurityType::*NO_AUTHENTICATION*)
        (t nil)))

(defclass kortex_driver::WifiSecurityType
  :super ros::object
  :slots ())

(defmethod kortex_driver::WifiSecurityType
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

(setf (get kortex_driver::WifiSecurityType :md5sum-) "758c13b30796cf34636bc4ccee207084")
(setf (get kortex_driver::WifiSecurityType :datatype-) "kortex_driver/WifiSecurityType")
(setf (get kortex_driver::WifiSecurityType :definition-)
      "
uint32 UNSPECIFIED_AUTHENTICATION = 0

uint32 WEP = 1

uint32 WPA2_PERSONAL = 2

uint32 WPA_PERSONAL = 4

uint32 NO_AUTHENTICATION = 8

")



(provide :kortex_driver/WifiSecurityType "758c13b30796cf34636bc4ccee207084")


