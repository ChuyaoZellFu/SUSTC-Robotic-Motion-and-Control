;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Resolution)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Resolution (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESOLUTION")
  (make-package "KORTEX_DRIVER::RESOLUTION"))

(in-package "ROS")
;;//! \htmlinclude Resolution.msg.html


(intern "*RESOLUTION_UNSPECIFIED*" (find-package "KORTEX_DRIVER::RESOLUTION"))
(shadow '*RESOLUTION_UNSPECIFIED* (find-package "KORTEX_DRIVER::RESOLUTION"))
(defconstant kortex_driver::Resolution::*RESOLUTION_UNSPECIFIED* 0)
(intern "*RESOLUTION_320X240*" (find-package "KORTEX_DRIVER::RESOLUTION"))
(shadow '*RESOLUTION_320X240* (find-package "KORTEX_DRIVER::RESOLUTION"))
(defconstant kortex_driver::Resolution::*RESOLUTION_320X240* 1)
(intern "*RESOLUTION_424X240*" (find-package "KORTEX_DRIVER::RESOLUTION"))
(shadow '*RESOLUTION_424X240* (find-package "KORTEX_DRIVER::RESOLUTION"))
(defconstant kortex_driver::Resolution::*RESOLUTION_424X240* 2)
(intern "*RESOLUTION_480X270*" (find-package "KORTEX_DRIVER::RESOLUTION"))
(shadow '*RESOLUTION_480X270* (find-package "KORTEX_DRIVER::RESOLUTION"))
(defconstant kortex_driver::Resolution::*RESOLUTION_480X270* 3)
(intern "*RESOLUTION_640X480*" (find-package "KORTEX_DRIVER::RESOLUTION"))
(shadow '*RESOLUTION_640X480* (find-package "KORTEX_DRIVER::RESOLUTION"))
(defconstant kortex_driver::Resolution::*RESOLUTION_640X480* 4)
(intern "*RESOLUTION_1280X720*" (find-package "KORTEX_DRIVER::RESOLUTION"))
(shadow '*RESOLUTION_1280X720* (find-package "KORTEX_DRIVER::RESOLUTION"))
(defconstant kortex_driver::Resolution::*RESOLUTION_1280X720* 5)
(intern "*RESOLUTION_1920X1080*" (find-package "KORTEX_DRIVER::RESOLUTION"))
(shadow '*RESOLUTION_1920X1080* (find-package "KORTEX_DRIVER::RESOLUTION"))
(defconstant kortex_driver::Resolution::*RESOLUTION_1920X1080* 6)

(defun kortex_driver::Resolution-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Resolution::*RESOLUTION_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::Resolution::*RESOLUTION_320x240*)
        ((= const 2) 'kortex_driver::Resolution::*RESOLUTION_424x240*)
        ((= const 3) 'kortex_driver::Resolution::*RESOLUTION_480x270*)
        ((= const 4) 'kortex_driver::Resolution::*RESOLUTION_640x480*)
        ((= const 5) 'kortex_driver::Resolution::*RESOLUTION_1280x720*)
        ((= const 6) 'kortex_driver::Resolution::*RESOLUTION_1920x1080*)
        (t nil)))

(defclass kortex_driver::Resolution
  :super ros::object
  :slots ())

(defmethod kortex_driver::Resolution
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

(setf (get kortex_driver::Resolution :md5sum-) "aa3fc86ed5b2ab08e6ab3385eebb39de")
(setf (get kortex_driver::Resolution :datatype-) "kortex_driver/Resolution")
(setf (get kortex_driver::Resolution :definition-)
      "
uint32 RESOLUTION_UNSPECIFIED = 0

uint32 RESOLUTION_320x240 = 1

uint32 RESOLUTION_424x240 = 2

uint32 RESOLUTION_480x270 = 3

uint32 RESOLUTION_640x480 = 4

uint32 RESOLUTION_1280x720 = 5

uint32 RESOLUTION_1920x1080 = 6

")



(provide :kortex_driver/Resolution "aa3fc86ed5b2ab08e6ab3385eebb39de")


