;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BridgeType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BridgeType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRIDGETYPE")
  (make-package "KORTEX_DRIVER::BRIDGETYPE"))

(in-package "ROS")
;;//! \htmlinclude BridgeType.msg.html


(intern "*BRIDGE_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::BRIDGETYPE"))
(shadow '*BRIDGE_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::BRIDGETYPE"))
(defconstant kortex_driver::BridgeType::*BRIDGE_TYPE_UNSPECIFIED* 0)
(intern "*BRIDGE_TYPE_UART*" (find-package "KORTEX_DRIVER::BRIDGETYPE"))
(shadow '*BRIDGE_TYPE_UART* (find-package "KORTEX_DRIVER::BRIDGETYPE"))
(defconstant kortex_driver::BridgeType::*BRIDGE_TYPE_UART* 1)
(intern "*BRIDGE_TYPE_TELNET*" (find-package "KORTEX_DRIVER::BRIDGETYPE"))
(shadow '*BRIDGE_TYPE_TELNET* (find-package "KORTEX_DRIVER::BRIDGETYPE"))
(defconstant kortex_driver::BridgeType::*BRIDGE_TYPE_TELNET* 2)

(defun kortex_driver::BridgeType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::BridgeType::*BRIDGE_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::BridgeType::*BRIDGE_TYPE_UART*)
        ((= const 2) 'kortex_driver::BridgeType::*BRIDGE_TYPE_TELNET*)
        (t nil)))

(defclass kortex_driver::BridgeType
  :super ros::object
  :slots ())

(defmethod kortex_driver::BridgeType
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

(setf (get kortex_driver::BridgeType :md5sum-) "b8e85d50bf3b32805a4cb6f83f1b2f4f")
(setf (get kortex_driver::BridgeType :datatype-) "kortex_driver/BridgeType")
(setf (get kortex_driver::BridgeType :definition-)
      "
uint32 BRIDGE_TYPE_UNSPECIFIED = 0

uint32 BRIDGE_TYPE_UART = 1

uint32 BRIDGE_TYPE_TELNET = 2

")



(provide :kortex_driver/BridgeType "b8e85d50bf3b32805a4cb6f83f1b2f4f")


