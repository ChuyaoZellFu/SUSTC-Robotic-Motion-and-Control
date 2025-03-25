;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WifiEncryptionType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WifiEncryptionType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE")
  (make-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))

(in-package "ROS")
;;//! \htmlinclude WifiEncryptionType.msg.html


(intern "*UNSPECIFIED_ENCRYPTION*" (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(shadow '*UNSPECIFIED_ENCRYPTION* (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(defconstant kortex_driver::WifiEncryptionType::*UNSPECIFIED_ENCRYPTION* 0)
(intern "*AES_ENCRYPTION*" (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(shadow '*AES_ENCRYPTION* (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(defconstant kortex_driver::WifiEncryptionType::*AES_ENCRYPTION* 1)
(intern "*TKIP_ENCRYPTION*" (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(shadow '*TKIP_ENCRYPTION* (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(defconstant kortex_driver::WifiEncryptionType::*TKIP_ENCRYPTION* 2)
(intern "*WEP_ENCRYPTION*" (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(shadow '*WEP_ENCRYPTION* (find-package "KORTEX_DRIVER::WIFIENCRYPTIONTYPE"))
(defconstant kortex_driver::WifiEncryptionType::*WEP_ENCRYPTION* 4)

(defun kortex_driver::WifiEncryptionType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::WifiEncryptionType::*UNSPECIFIED_ENCRYPTION*)
        ((= const 1) 'kortex_driver::WifiEncryptionType::*AES_ENCRYPTION*)
        ((= const 2) 'kortex_driver::WifiEncryptionType::*TKIP_ENCRYPTION*)
        ((= const 4) 'kortex_driver::WifiEncryptionType::*WEP_ENCRYPTION*)
        (t nil)))

(defclass kortex_driver::WifiEncryptionType
  :super ros::object
  :slots ())

(defmethod kortex_driver::WifiEncryptionType
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

(setf (get kortex_driver::WifiEncryptionType :md5sum-) "5312088e9fe010ba1511377fa25b980d")
(setf (get kortex_driver::WifiEncryptionType :datatype-) "kortex_driver/WifiEncryptionType")
(setf (get kortex_driver::WifiEncryptionType :definition-)
      "
uint32 UNSPECIFIED_ENCRYPTION = 0

uint32 AES_ENCRYPTION = 1

uint32 TKIP_ENCRYPTION = 2

uint32 WEP_ENCRYPTION = 4

")



(provide :kortex_driver/WifiEncryptionType "5312088e9fe010ba1511377fa25b980d")


