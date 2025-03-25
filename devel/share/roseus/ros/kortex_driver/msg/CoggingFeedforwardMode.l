;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CoggingFeedforwardMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CoggingFeedforwardMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE")
  (make-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE"))

(in-package "ROS")
;;//! \htmlinclude CoggingFeedforwardMode.msg.html


(intern "*FEEDFORWARD_OFF*" (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE"))
(shadow '*FEEDFORWARD_OFF* (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE"))
(defconstant kortex_driver::CoggingFeedforwardMode::*FEEDFORWARD_OFF* 0)
(intern "*FEEDFORWARD_ADAPTIVE*" (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE"))
(shadow '*FEEDFORWARD_ADAPTIVE* (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE"))
(defconstant kortex_driver::CoggingFeedforwardMode::*FEEDFORWARD_ADAPTIVE* 1)
(intern "*FEEDFORWARD_CALIBRATED*" (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE"))
(shadow '*FEEDFORWARD_CALIBRATED* (find-package "KORTEX_DRIVER::COGGINGFEEDFORWARDMODE"))
(defconstant kortex_driver::CoggingFeedforwardMode::*FEEDFORWARD_CALIBRATED* 2)

(defun kortex_driver::CoggingFeedforwardMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::CoggingFeedforwardMode::*FEEDFORWARD_OFF*)
        ((= const 1) 'kortex_driver::CoggingFeedforwardMode::*FEEDFORWARD_ADAPTIVE*)
        ((= const 2) 'kortex_driver::CoggingFeedforwardMode::*FEEDFORWARD_CALIBRATED*)
        (t nil)))

(defclass kortex_driver::CoggingFeedforwardMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::CoggingFeedforwardMode
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

(setf (get kortex_driver::CoggingFeedforwardMode :md5sum-) "040e60ae33515a7c8082e377632750e6")
(setf (get kortex_driver::CoggingFeedforwardMode :datatype-) "kortex_driver/CoggingFeedforwardMode")
(setf (get kortex_driver::CoggingFeedforwardMode :definition-)
      "
uint32 FEEDFORWARD_OFF = 0

uint32 FEEDFORWARD_ADAPTIVE = 1

uint32 FEEDFORWARD_CALIBRATED = 2

")



(provide :kortex_driver/CoggingFeedforwardMode "040e60ae33515a7c8082e377632750e6")


