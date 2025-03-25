;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BridgeStatus)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BridgeStatus (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRIDGESTATUS")
  (make-package "KORTEX_DRIVER::BRIDGESTATUS"))

(in-package "ROS")
;;//! \htmlinclude BridgeStatus.msg.html


(intern "*BRIDGE_STATUS_OK*" (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(shadow '*BRIDGE_STATUS_OK* (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(defconstant kortex_driver::BridgeStatus::*BRIDGE_STATUS_OK* 0)
(intern "*BRIDGE_STATUS_OUTP_UNAVAILABLE*" (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(shadow '*BRIDGE_STATUS_OUTP_UNAVAILABLE* (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(defconstant kortex_driver::BridgeStatus::*BRIDGE_STATUS_OUTP_UNAVAILABLE* 1)
(intern "*BRIDGE_STATUS_UNKNOWN_DEVID*" (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(shadow '*BRIDGE_STATUS_UNKNOWN_DEVID* (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(defconstant kortex_driver::BridgeStatus::*BRIDGE_STATUS_UNKNOWN_DEVID* 2)
(intern "*BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE*" (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(shadow '*BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE* (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(defconstant kortex_driver::BridgeStatus::*BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE* 3)
(intern "*BRIDGE_STATUS_NOT_FOUND*" (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(shadow '*BRIDGE_STATUS_NOT_FOUND* (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(defconstant kortex_driver::BridgeStatus::*BRIDGE_STATUS_NOT_FOUND* 4)
(intern "*BRIDGE_STATUS_NOT_INITIALIZED*" (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(shadow '*BRIDGE_STATUS_NOT_INITIALIZED* (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(defconstant kortex_driver::BridgeStatus::*BRIDGE_STATUS_NOT_INITIALIZED* 5)
(intern "*BRIDGE_STATUS_UNKNOWN*" (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(shadow '*BRIDGE_STATUS_UNKNOWN* (find-package "KORTEX_DRIVER::BRIDGESTATUS"))
(defconstant kortex_driver::BridgeStatus::*BRIDGE_STATUS_UNKNOWN* 6)

(defun kortex_driver::BridgeStatus-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::BridgeStatus::*BRIDGE_STATUS_OK*)
        ((= const 1) 'kortex_driver::BridgeStatus::*BRIDGE_STATUS_OUTP_UNAVAILABLE*)
        ((= const 2) 'kortex_driver::BridgeStatus::*BRIDGE_STATUS_UNKNOWN_DEVID*)
        ((= const 3) 'kortex_driver::BridgeStatus::*BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE*)
        ((= const 4) 'kortex_driver::BridgeStatus::*BRIDGE_STATUS_NOT_FOUND*)
        ((= const 5) 'kortex_driver::BridgeStatus::*BRIDGE_STATUS_NOT_INITIALIZED*)
        ((= const 6) 'kortex_driver::BridgeStatus::*BRIDGE_STATUS_UNKNOWN*)
        (t nil)))

(defclass kortex_driver::BridgeStatus
  :super ros::object
  :slots ())

(defmethod kortex_driver::BridgeStatus
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

(setf (get kortex_driver::BridgeStatus :md5sum-) "9c6586b899b39a3f8a12bc04700166a0")
(setf (get kortex_driver::BridgeStatus :datatype-) "kortex_driver/BridgeStatus")
(setf (get kortex_driver::BridgeStatus :definition-)
      "
uint32 BRIDGE_STATUS_OK = 0

uint32 BRIDGE_STATUS_OUTP_UNAVAILABLE = 1

uint32 BRIDGE_STATUS_UNKNOWN_DEVID = 2

uint32 BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE = 3

uint32 BRIDGE_STATUS_NOT_FOUND = 4

uint32 BRIDGE_STATUS_NOT_INITIALIZED = 5

uint32 BRIDGE_STATUS_UNKNOWN = 6

")



(provide :kortex_driver/BridgeStatus "9c6586b899b39a3f8a12bc04700166a0")


