;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FrameRate)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FrameRate (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FRAMERATE")
  (make-package "KORTEX_DRIVER::FRAMERATE"))

(in-package "ROS")
;;//! \htmlinclude FrameRate.msg.html


(intern "*FRAMERATE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::FRAMERATE"))
(shadow '*FRAMERATE_UNSPECIFIED* (find-package "KORTEX_DRIVER::FRAMERATE"))
(defconstant kortex_driver::FrameRate::*FRAMERATE_UNSPECIFIED* 0)
(intern "*FRAMERATE_6_FPS*" (find-package "KORTEX_DRIVER::FRAMERATE"))
(shadow '*FRAMERATE_6_FPS* (find-package "KORTEX_DRIVER::FRAMERATE"))
(defconstant kortex_driver::FrameRate::*FRAMERATE_6_FPS* 1)
(intern "*FRAMERATE_15_FPS*" (find-package "KORTEX_DRIVER::FRAMERATE"))
(shadow '*FRAMERATE_15_FPS* (find-package "KORTEX_DRIVER::FRAMERATE"))
(defconstant kortex_driver::FrameRate::*FRAMERATE_15_FPS* 2)
(intern "*FRAMERATE_30_FPS*" (find-package "KORTEX_DRIVER::FRAMERATE"))
(shadow '*FRAMERATE_30_FPS* (find-package "KORTEX_DRIVER::FRAMERATE"))
(defconstant kortex_driver::FrameRate::*FRAMERATE_30_FPS* 3)

(defun kortex_driver::FrameRate-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::FrameRate::*FRAMERATE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::FrameRate::*FRAMERATE_6_FPS*)
        ((= const 2) 'kortex_driver::FrameRate::*FRAMERATE_15_FPS*)
        ((= const 3) 'kortex_driver::FrameRate::*FRAMERATE_30_FPS*)
        (t nil)))

(defclass kortex_driver::FrameRate
  :super ros::object
  :slots ())

(defmethod kortex_driver::FrameRate
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

(setf (get kortex_driver::FrameRate :md5sum-) "9400397c1a84f4718dc33b3b7f3b5df4")
(setf (get kortex_driver::FrameRate :datatype-) "kortex_driver/FrameRate")
(setf (get kortex_driver::FrameRate :definition-)
      "
uint32 FRAMERATE_UNSPECIFIED = 0

uint32 FRAMERATE_6_FPS = 1

uint32 FRAMERATE_15_FPS = 2

uint32 FRAMERATE_30_FPS = 3

")



(provide :kortex_driver/FrameRate "9400397c1a84f4718dc33b3b7f3b5df4")


