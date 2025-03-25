;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ErrorCodes)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ErrorCodes (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ERRORCODES")
  (make-package "KORTEX_DRIVER::ERRORCODES"))

(in-package "ROS")
;;//! \htmlinclude ErrorCodes.msg.html


(intern "*ERROR_NONE*" (find-package "KORTEX_DRIVER::ERRORCODES"))
(shadow '*ERROR_NONE* (find-package "KORTEX_DRIVER::ERRORCODES"))
(defconstant kortex_driver::ErrorCodes::*ERROR_NONE* 0)
(intern "*ERROR_PROTOCOL_SERVER*" (find-package "KORTEX_DRIVER::ERRORCODES"))
(shadow '*ERROR_PROTOCOL_SERVER* (find-package "KORTEX_DRIVER::ERRORCODES"))
(defconstant kortex_driver::ErrorCodes::*ERROR_PROTOCOL_SERVER* 1)
(intern "*ERROR_PROTOCOL_CLIENT*" (find-package "KORTEX_DRIVER::ERRORCODES"))
(shadow '*ERROR_PROTOCOL_CLIENT* (find-package "KORTEX_DRIVER::ERRORCODES"))
(defconstant kortex_driver::ErrorCodes::*ERROR_PROTOCOL_CLIENT* 2)
(intern "*ERROR_DEVICE*" (find-package "KORTEX_DRIVER::ERRORCODES"))
(shadow '*ERROR_DEVICE* (find-package "KORTEX_DRIVER::ERRORCODES"))
(defconstant kortex_driver::ErrorCodes::*ERROR_DEVICE* 3)
(intern "*ERROR_INTERNAL*" (find-package "KORTEX_DRIVER::ERRORCODES"))
(shadow '*ERROR_INTERNAL* (find-package "KORTEX_DRIVER::ERRORCODES"))
(defconstant kortex_driver::ErrorCodes::*ERROR_INTERNAL* 4)

(defun kortex_driver::ErrorCodes-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ErrorCodes::*ERROR_NONE*)
        ((= const 1) 'kortex_driver::ErrorCodes::*ERROR_PROTOCOL_SERVER*)
        ((= const 2) 'kortex_driver::ErrorCodes::*ERROR_PROTOCOL_CLIENT*)
        ((= const 3) 'kortex_driver::ErrorCodes::*ERROR_DEVICE*)
        ((= const 4) 'kortex_driver::ErrorCodes::*ERROR_INTERNAL*)
        (t nil)))

(defclass kortex_driver::ErrorCodes
  :super ros::object
  :slots ())

(defmethod kortex_driver::ErrorCodes
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

(setf (get kortex_driver::ErrorCodes :md5sum-) "cb78b6e4994aaf216b83b68ecaa14cbd")
(setf (get kortex_driver::ErrorCodes :datatype-) "kortex_driver/ErrorCodes")
(setf (get kortex_driver::ErrorCodes :definition-)
      "
uint32 ERROR_NONE = 0

uint32 ERROR_PROTOCOL_SERVER = 1

uint32 ERROR_PROTOCOL_CLIENT = 2

uint32 ERROR_DEVICE = 3

uint32 ERROR_INTERNAL = 4

")



(provide :kortex_driver/ErrorCodes "cb78b6e4994aaf216b83b68ecaa14cbd")


