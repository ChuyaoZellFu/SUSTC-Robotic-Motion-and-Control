;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::NetworkEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'NetworkEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::NETWORKEVENT")
  (make-package "KORTEX_DRIVER::NETWORKEVENT"))

(in-package "ROS")
;;//! \htmlinclude NetworkEvent.msg.html


(intern "*UNSPECIFIED_NETWORK_EVENT*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*UNSPECIFIED_NETWORK_EVENT* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*UNSPECIFIED_NETWORK_EVENT* 0)
(intern "*WIFI_CONNECTED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_CONNECTED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_CONNECTED* 1)
(intern "*WIFI_DISCONNECTED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_DISCONNECTED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_DISCONNECTED* 2)
(intern "*WIFI_SCAN_STARTED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_SCAN_STARTED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_SCAN_STARTED* 3)
(intern "*WIFI_SCAN_RESULTS*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_SCAN_RESULTS* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_SCAN_RESULTS* 4)
(intern "*WIFI_SCAN_FAILED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_SCAN_FAILED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_SCAN_FAILED* 5)
(intern "*WIFI_NOT_FOUND*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_NOT_FOUND* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_NOT_FOUND* 6)
(intern "*WIFI_ASSOC_REJECTED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_ASSOC_REJECTED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_ASSOC_REJECTED* 7)
(intern "*WIFI_AUTH_WRONG_KEY*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_AUTH_WRONG_KEY* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_AUTH_WRONG_KEY* 8)
(intern "*WIFI_AUTH_CONN_FAILED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_AUTH_CONN_FAILED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_AUTH_CONN_FAILED* 9)
(intern "*WIFI_AUTH_FAILED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_AUTH_FAILED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_AUTH_FAILED* 10)
(intern "*WIFI_ENABLED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_ENABLED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_ENABLED* 11)
(intern "*WIFI_DISABLED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*WIFI_DISABLED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*WIFI_DISABLED* 12)
(intern "*BLUETOOTH_ENABLED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*BLUETOOTH_ENABLED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*BLUETOOTH_ENABLED* 13)
(intern "*BLUETOOTH_DISABLED*" (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(shadow '*BLUETOOTH_DISABLED* (find-package "KORTEX_DRIVER::NETWORKEVENT"))
(defconstant kortex_driver::NetworkEvent::*BLUETOOTH_DISABLED* 14)

(defun kortex_driver::NetworkEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::NetworkEvent::*UNSPECIFIED_NETWORK_EVENT*)
        ((= const 1) 'kortex_driver::NetworkEvent::*WIFI_CONNECTED*)
        ((= const 2) 'kortex_driver::NetworkEvent::*WIFI_DISCONNECTED*)
        ((= const 3) 'kortex_driver::NetworkEvent::*WIFI_SCAN_STARTED*)
        ((= const 4) 'kortex_driver::NetworkEvent::*WIFI_SCAN_RESULTS*)
        ((= const 5) 'kortex_driver::NetworkEvent::*WIFI_SCAN_FAILED*)
        ((= const 6) 'kortex_driver::NetworkEvent::*WIFI_NOT_FOUND*)
        ((= const 7) 'kortex_driver::NetworkEvent::*WIFI_ASSOC_REJECTED*)
        ((= const 8) 'kortex_driver::NetworkEvent::*WIFI_AUTH_WRONG_KEY*)
        ((= const 9) 'kortex_driver::NetworkEvent::*WIFI_AUTH_CONN_FAILED*)
        ((= const 10) 'kortex_driver::NetworkEvent::*WIFI_AUTH_FAILED*)
        ((= const 11) 'kortex_driver::NetworkEvent::*WIFI_ENABLED*)
        ((= const 12) 'kortex_driver::NetworkEvent::*WIFI_DISABLED*)
        ((= const 13) 'kortex_driver::NetworkEvent::*BLUETOOTH_ENABLED*)
        ((= const 14) 'kortex_driver::NetworkEvent::*BLUETOOTH_DISABLED*)
        (t nil)))

(defclass kortex_driver::NetworkEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::NetworkEvent
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

(setf (get kortex_driver::NetworkEvent :md5sum-) "342adcc6d6071ab5d5d0b6d3b3ff3e84")
(setf (get kortex_driver::NetworkEvent :datatype-) "kortex_driver/NetworkEvent")
(setf (get kortex_driver::NetworkEvent :definition-)
      "
uint32 UNSPECIFIED_NETWORK_EVENT = 0

uint32 WIFI_CONNECTED = 1

uint32 WIFI_DISCONNECTED = 2

uint32 WIFI_SCAN_STARTED = 3

uint32 WIFI_SCAN_RESULTS = 4

uint32 WIFI_SCAN_FAILED = 5

uint32 WIFI_NOT_FOUND = 6

uint32 WIFI_ASSOC_REJECTED = 7

uint32 WIFI_AUTH_WRONG_KEY = 8

uint32 WIFI_AUTH_CONN_FAILED = 9

uint32 WIFI_AUTH_FAILED = 10

uint32 WIFI_ENABLED = 11

uint32 WIFI_DISABLED = 12

uint32 BLUETOOTH_ENABLED = 13

uint32 BLUETOOTH_DISABLED = 14

")



(provide :kortex_driver/NetworkEvent "342adcc6d6071ab5d5d0b6d3b3ff3e84")


