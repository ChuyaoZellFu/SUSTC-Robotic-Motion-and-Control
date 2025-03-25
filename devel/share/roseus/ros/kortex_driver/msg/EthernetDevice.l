;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EthernetDevice)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EthernetDevice (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ETHERNETDEVICE")
  (make-package "KORTEX_DRIVER::ETHERNETDEVICE"))

(in-package "ROS")
;;//! \htmlinclude EthernetDevice.msg.html


(intern "*ETHERNET_DEVICE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::ETHERNETDEVICE"))
(shadow '*ETHERNET_DEVICE_UNSPECIFIED* (find-package "KORTEX_DRIVER::ETHERNETDEVICE"))
(defconstant kortex_driver::EthernetDevice::*ETHERNET_DEVICE_UNSPECIFIED* 0)
(intern "*ETHERNET_DEVICE_EXPANSION*" (find-package "KORTEX_DRIVER::ETHERNETDEVICE"))
(shadow '*ETHERNET_DEVICE_EXPANSION* (find-package "KORTEX_DRIVER::ETHERNETDEVICE"))
(defconstant kortex_driver::EthernetDevice::*ETHERNET_DEVICE_EXPANSION* 1)

(defun kortex_driver::EthernetDevice-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::EthernetDevice::*ETHERNET_DEVICE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::EthernetDevice::*ETHERNET_DEVICE_EXPANSION*)
        (t nil)))

(defclass kortex_driver::EthernetDevice
  :super ros::object
  :slots ())

(defmethod kortex_driver::EthernetDevice
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

(setf (get kortex_driver::EthernetDevice :md5sum-) "cbad10c4e89ab17bbf73ffd6e85d4b44")
(setf (get kortex_driver::EthernetDevice :datatype-) "kortex_driver/EthernetDevice")
(setf (get kortex_driver::EthernetDevice :definition-)
      "
uint32 ETHERNET_DEVICE_UNSPECIFIED = 0

uint32 ETHERNET_DEVICE_EXPANSION = 1

")



(provide :kortex_driver/EthernetDevice "cbad10c4e89ab17bbf73ffd6e85d4b44")


