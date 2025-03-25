;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_CapSenseMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_CapSenseMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE")
  (make-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))

(in-package "ROS")
;;//! \htmlinclude Base_CapSenseMode.msg.html


(intern "*RESERVED*" (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(shadow '*RESERVED* (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(defconstant kortex_driver::Base_CapSenseMode::*RESERVED* 0)
(intern "*INACTIVE*" (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(shadow '*INACTIVE* (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(defconstant kortex_driver::Base_CapSenseMode::*INACTIVE* 1)
(intern "*ACTIVE_AUTO_THRESHOLD*" (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(shadow '*ACTIVE_AUTO_THRESHOLD* (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(defconstant kortex_driver::Base_CapSenseMode::*ACTIVE_AUTO_THRESHOLD* 2)
(intern "*ACTIVE_NOISE_ATT*" (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(shadow '*ACTIVE_NOISE_ATT* (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(defconstant kortex_driver::Base_CapSenseMode::*ACTIVE_NOISE_ATT* 4)
(intern "*ACTIVE_NORMAL*" (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(shadow '*ACTIVE_NORMAL* (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(defconstant kortex_driver::Base_CapSenseMode::*ACTIVE_NORMAL* 5)
(intern "*CONFIGURATION*" (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(shadow '*CONFIGURATION* (find-package "KORTEX_DRIVER::BASE_CAPSENSEMODE"))
(defconstant kortex_driver::Base_CapSenseMode::*CONFIGURATION* 6)

(defun kortex_driver::Base_CapSenseMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Base_CapSenseMode::*RESERVED*)
        ((= const 1) 'kortex_driver::Base_CapSenseMode::*INACTIVE*)
        ((= const 2) 'kortex_driver::Base_CapSenseMode::*ACTIVE_AUTO_THRESHOLD*)
        ((= const 4) 'kortex_driver::Base_CapSenseMode::*ACTIVE_NOISE_ATT*)
        ((= const 5) 'kortex_driver::Base_CapSenseMode::*ACTIVE_NORMAL*)
        ((= const 6) 'kortex_driver::Base_CapSenseMode::*CONFIGURATION*)
        (t nil)))

(defclass kortex_driver::Base_CapSenseMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::Base_CapSenseMode
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

(setf (get kortex_driver::Base_CapSenseMode :md5sum-) "ab37ae8a57a132b00bf1d50bed534551")
(setf (get kortex_driver::Base_CapSenseMode :datatype-) "kortex_driver/Base_CapSenseMode")
(setf (get kortex_driver::Base_CapSenseMode :definition-)
      "
uint32 RESERVED = 0

uint32 INACTIVE = 1

uint32 ACTIVE_AUTO_THRESHOLD = 2

uint32 ACTIVE_NOISE_ATT = 4

uint32 ACTIVE_NORMAL = 5

uint32 CONFIGURATION = 6

")



(provide :kortex_driver/Base_CapSenseMode "ab37ae8a57a132b00bf1d50bed534551")


