;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SignalQuality)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SignalQuality (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SIGNALQUALITY")
  (make-package "KORTEX_DRIVER::SIGNALQUALITY"))

(in-package "ROS")
;;//! \htmlinclude SignalQuality.msg.html


(intern "*UNSPECIFIED_SIGNAL_QUALITY*" (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(shadow '*UNSPECIFIED_SIGNAL_QUALITY* (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(defconstant kortex_driver::SignalQuality::*UNSPECIFIED_SIGNAL_QUALITY* 0)
(intern "*POOR*" (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(shadow '*POOR* (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(defconstant kortex_driver::SignalQuality::*POOR* 1)
(intern "*FAIR*" (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(shadow '*FAIR* (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(defconstant kortex_driver::SignalQuality::*FAIR* 2)
(intern "*GOOD*" (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(shadow '*GOOD* (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(defconstant kortex_driver::SignalQuality::*GOOD* 3)
(intern "*EXCELLENT*" (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(shadow '*EXCELLENT* (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(defconstant kortex_driver::SignalQuality::*EXCELLENT* 4)
(intern "*NONE*" (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(shadow '*NONE* (find-package "KORTEX_DRIVER::SIGNALQUALITY"))
(defconstant kortex_driver::SignalQuality::*NONE* 5)

(defun kortex_driver::SignalQuality-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::SignalQuality::*UNSPECIFIED_SIGNAL_QUALITY*)
        ((= const 1) 'kortex_driver::SignalQuality::*POOR*)
        ((= const 2) 'kortex_driver::SignalQuality::*FAIR*)
        ((= const 3) 'kortex_driver::SignalQuality::*GOOD*)
        ((= const 4) 'kortex_driver::SignalQuality::*EXCELLENT*)
        ((= const 5) 'kortex_driver::SignalQuality::*NONE*)
        (t nil)))

(defclass kortex_driver::SignalQuality
  :super ros::object
  :slots ())

(defmethod kortex_driver::SignalQuality
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

(setf (get kortex_driver::SignalQuality :md5sum-) "080b7090353f86dd3efade43f40ad044")
(setf (get kortex_driver::SignalQuality :datatype-) "kortex_driver/SignalQuality")
(setf (get kortex_driver::SignalQuality :definition-)
      "
uint32 UNSPECIFIED_SIGNAL_QUALITY = 0

uint32 POOR = 1

uint32 FAIR = 2

uint32 GOOD = 3

uint32 EXCELLENT = 4

uint32 NONE = 5

")



(provide :kortex_driver/SignalQuality "080b7090353f86dd3efade43f40ad044")


