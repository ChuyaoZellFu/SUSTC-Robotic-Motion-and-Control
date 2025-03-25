;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EthernetDuplex)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EthernetDuplex (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ETHERNETDUPLEX")
  (make-package "KORTEX_DRIVER::ETHERNETDUPLEX"))

(in-package "ROS")
;;//! \htmlinclude EthernetDuplex.msg.html


(intern "*ETHERNET_DUPLEX_UNSPECIFIED*" (find-package "KORTEX_DRIVER::ETHERNETDUPLEX"))
(shadow '*ETHERNET_DUPLEX_UNSPECIFIED* (find-package "KORTEX_DRIVER::ETHERNETDUPLEX"))
(defconstant kortex_driver::EthernetDuplex::*ETHERNET_DUPLEX_UNSPECIFIED* 0)
(intern "*ETHERNET_DUPLEX_HALF*" (find-package "KORTEX_DRIVER::ETHERNETDUPLEX"))
(shadow '*ETHERNET_DUPLEX_HALF* (find-package "KORTEX_DRIVER::ETHERNETDUPLEX"))
(defconstant kortex_driver::EthernetDuplex::*ETHERNET_DUPLEX_HALF* 1)
(intern "*ETHERNET_DUPLEX_FULL*" (find-package "KORTEX_DRIVER::ETHERNETDUPLEX"))
(shadow '*ETHERNET_DUPLEX_FULL* (find-package "KORTEX_DRIVER::ETHERNETDUPLEX"))
(defconstant kortex_driver::EthernetDuplex::*ETHERNET_DUPLEX_FULL* 2)

(defun kortex_driver::EthernetDuplex-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::EthernetDuplex::*ETHERNET_DUPLEX_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::EthernetDuplex::*ETHERNET_DUPLEX_HALF*)
        ((= const 2) 'kortex_driver::EthernetDuplex::*ETHERNET_DUPLEX_FULL*)
        (t nil)))

(defclass kortex_driver::EthernetDuplex
  :super ros::object
  :slots ())

(defmethod kortex_driver::EthernetDuplex
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

(setf (get kortex_driver::EthernetDuplex :md5sum-) "cd452927d77476fb60f0cc73e416a7ef")
(setf (get kortex_driver::EthernetDuplex :datatype-) "kortex_driver/EthernetDuplex")
(setf (get kortex_driver::EthernetDuplex :definition-)
      "
uint32 ETHERNET_DUPLEX_UNSPECIFIED = 0

uint32 ETHERNET_DUPLEX_HALF = 1

uint32 ETHERNET_DUPLEX_FULL = 2

")



(provide :kortex_driver/EthernetDuplex "cd452927d77476fb60f0cc73e416a7ef")


