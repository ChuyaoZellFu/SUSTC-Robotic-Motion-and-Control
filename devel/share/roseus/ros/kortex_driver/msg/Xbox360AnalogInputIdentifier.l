;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Xbox360AnalogInputIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Xbox360AnalogInputIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER")
  (make-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude Xbox360AnalogInputIdentifier.msg.html


(intern "*UNSPECIFIED_XBOX360_ANALOG*" (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(shadow '*UNSPECIFIED_XBOX360_ANALOG* (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360AnalogInputIdentifier::*UNSPECIFIED_XBOX360_ANALOG* 0)
(intern "*XBOX360_THUMB_LEFT_X*" (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(shadow '*XBOX360_THUMB_LEFT_X* (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_LEFT_X* 1)
(intern "*XBOX360_THUMB_LEFT_Y*" (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(shadow '*XBOX360_THUMB_LEFT_Y* (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_LEFT_Y* 2)
(intern "*XBOX360_THUMB_RIGHT_X*" (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(shadow '*XBOX360_THUMB_RIGHT_X* (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_RIGHT_X* 3)
(intern "*XBOX360_THUMB_RIGHT_Y*" (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(shadow '*XBOX360_THUMB_RIGHT_Y* (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_RIGHT_Y* 4)
(intern "*XBOX360_TRIGGER_LEFT*" (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(shadow '*XBOX360_TRIGGER_LEFT* (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_TRIGGER_LEFT* 5)
(intern "*XBOX360_TRIGGER_RIGHT*" (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(shadow '*XBOX360_TRIGGER_RIGHT* (find-package "KORTEX_DRIVER::XBOX360ANALOGINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_TRIGGER_RIGHT* 6)

(defun kortex_driver::Xbox360AnalogInputIdentifier-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Xbox360AnalogInputIdentifier::*UNSPECIFIED_XBOX360_ANALOG*)
        ((= const 1) 'kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_LEFT_X*)
        ((= const 2) 'kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_LEFT_Y*)
        ((= const 3) 'kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_RIGHT_X*)
        ((= const 4) 'kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_THUMB_RIGHT_Y*)
        ((= const 5) 'kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_TRIGGER_LEFT*)
        ((= const 6) 'kortex_driver::Xbox360AnalogInputIdentifier::*XBOX360_TRIGGER_RIGHT*)
        (t nil)))

(defclass kortex_driver::Xbox360AnalogInputIdentifier
  :super ros::object
  :slots ())

(defmethod kortex_driver::Xbox360AnalogInputIdentifier
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

(setf (get kortex_driver::Xbox360AnalogInputIdentifier :md5sum-) "bdf16a9285171f4643549a6ec207283d")
(setf (get kortex_driver::Xbox360AnalogInputIdentifier :datatype-) "kortex_driver/Xbox360AnalogInputIdentifier")
(setf (get kortex_driver::Xbox360AnalogInputIdentifier :definition-)
      "
uint32 UNSPECIFIED_XBOX360_ANALOG = 0

uint32 XBOX360_THUMB_LEFT_X = 1

uint32 XBOX360_THUMB_LEFT_Y = 2

uint32 XBOX360_THUMB_RIGHT_X = 3

uint32 XBOX360_THUMB_RIGHT_Y = 4

uint32 XBOX360_TRIGGER_LEFT = 5

uint32 XBOX360_TRIGGER_RIGHT = 6

")



(provide :kortex_driver/Xbox360AnalogInputIdentifier "bdf16a9285171f4643549a6ec207283d")


