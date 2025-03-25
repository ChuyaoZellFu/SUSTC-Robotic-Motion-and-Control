;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BitRate)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BitRate (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BITRATE")
  (make-package "KORTEX_DRIVER::BITRATE"))

(in-package "ROS")
;;//! \htmlinclude BitRate.msg.html


(intern "*BITRATE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::BITRATE"))
(shadow '*BITRATE_UNSPECIFIED* (find-package "KORTEX_DRIVER::BITRATE"))
(defconstant kortex_driver::BitRate::*BITRATE_UNSPECIFIED* 0)
(intern "*BITRATE_10_MBPS*" (find-package "KORTEX_DRIVER::BITRATE"))
(shadow '*BITRATE_10_MBPS* (find-package "KORTEX_DRIVER::BITRATE"))
(defconstant kortex_driver::BitRate::*BITRATE_10_MBPS* 1)
(intern "*BITRATE_15_MBPS*" (find-package "KORTEX_DRIVER::BITRATE"))
(shadow '*BITRATE_15_MBPS* (find-package "KORTEX_DRIVER::BITRATE"))
(defconstant kortex_driver::BitRate::*BITRATE_15_MBPS* 2)
(intern "*BITRATE_20_MBPS*" (find-package "KORTEX_DRIVER::BITRATE"))
(shadow '*BITRATE_20_MBPS* (find-package "KORTEX_DRIVER::BITRATE"))
(defconstant kortex_driver::BitRate::*BITRATE_20_MBPS* 3)
(intern "*BITRATE_25_MBPS*" (find-package "KORTEX_DRIVER::BITRATE"))
(shadow '*BITRATE_25_MBPS* (find-package "KORTEX_DRIVER::BITRATE"))
(defconstant kortex_driver::BitRate::*BITRATE_25_MBPS* 4)

(defun kortex_driver::BitRate-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::BitRate::*BITRATE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::BitRate::*BITRATE_10_MBPS*)
        ((= const 2) 'kortex_driver::BitRate::*BITRATE_15_MBPS*)
        ((= const 3) 'kortex_driver::BitRate::*BITRATE_20_MBPS*)
        ((= const 4) 'kortex_driver::BitRate::*BITRATE_25_MBPS*)
        (t nil)))

(defclass kortex_driver::BitRate
  :super ros::object
  :slots ())

(defmethod kortex_driver::BitRate
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

(setf (get kortex_driver::BitRate :md5sum-) "472a4e4218d43df9cfed86c61a2322e6")
(setf (get kortex_driver::BitRate :datatype-) "kortex_driver/BitRate")
(setf (get kortex_driver::BitRate :definition-)
      "
uint32 BITRATE_UNSPECIFIED = 0

uint32 BITRATE_10_MBPS = 1

uint32 BITRATE_15_MBPS = 2

uint32 BITRATE_20_MBPS = 3

uint32 BITRATE_25_MBPS = 4

")



(provide :kortex_driver/BitRate "472a4e4218d43df9cfed86c61a2322e6")


