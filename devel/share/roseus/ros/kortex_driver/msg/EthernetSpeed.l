;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EthernetSpeed)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EthernetSpeed (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ETHERNETSPEED")
  (make-package "KORTEX_DRIVER::ETHERNETSPEED"))

(in-package "ROS")
;;//! \htmlinclude EthernetSpeed.msg.html


(intern "*ETHERNET_SPEED_UNSPECIFIED*" (find-package "KORTEX_DRIVER::ETHERNETSPEED"))
(shadow '*ETHERNET_SPEED_UNSPECIFIED* (find-package "KORTEX_DRIVER::ETHERNETSPEED"))
(defconstant kortex_driver::EthernetSpeed::*ETHERNET_SPEED_UNSPECIFIED* 0)
(intern "*ETHERNET_SPEED_10M*" (find-package "KORTEX_DRIVER::ETHERNETSPEED"))
(shadow '*ETHERNET_SPEED_10M* (find-package "KORTEX_DRIVER::ETHERNETSPEED"))
(defconstant kortex_driver::EthernetSpeed::*ETHERNET_SPEED_10M* 1)
(intern "*ETHERNET_SPEED_100M*" (find-package "KORTEX_DRIVER::ETHERNETSPEED"))
(shadow '*ETHERNET_SPEED_100M* (find-package "KORTEX_DRIVER::ETHERNETSPEED"))
(defconstant kortex_driver::EthernetSpeed::*ETHERNET_SPEED_100M* 2)

(defun kortex_driver::EthernetSpeed-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::EthernetSpeed::*ETHERNET_SPEED_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::EthernetSpeed::*ETHERNET_SPEED_10M*)
        ((= const 2) 'kortex_driver::EthernetSpeed::*ETHERNET_SPEED_100M*)
        (t nil)))

(defclass kortex_driver::EthernetSpeed
  :super ros::object
  :slots ())

(defmethod kortex_driver::EthernetSpeed
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

(setf (get kortex_driver::EthernetSpeed :md5sum-) "03143ee942f3291a850b685ab265f926")
(setf (get kortex_driver::EthernetSpeed :datatype-) "kortex_driver/EthernetSpeed")
(setf (get kortex_driver::EthernetSpeed :definition-)
      "
uint32 ETHERNET_SPEED_UNSPECIFIED = 0

uint32 ETHERNET_SPEED_10M = 1

uint32 ETHERNET_SPEED_100M = 2

")



(provide :kortex_driver/EthernetSpeed "03143ee942f3291a850b685ab265f926")


