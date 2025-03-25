;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Xbox360DigitalInputIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Xbox360DigitalInputIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER")
  (make-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude Xbox360DigitalInputIdentifier.msg.html


(intern "*UNSPECIFIED_XBOX360_DIGITAL*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*UNSPECIFIED_XBOX360_DIGITAL* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*UNSPECIFIED_XBOX360_DIGITAL* 0)
(intern "*XBOX360_PAD_UP*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_PAD_UP* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_UP* 1)
(intern "*XBOX360_PAD_DOWN*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_PAD_DOWN* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_DOWN* 2)
(intern "*XBOX360_PAD_LEFT*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_PAD_LEFT* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_LEFT* 3)
(intern "*XBOX360_PAD_RIGHT*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_PAD_RIGHT* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_RIGHT* 4)
(intern "*XBOX360_FILE_BUTTON_START*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_FILE_BUTTON_START* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_FILE_BUTTON_START* 5)
(intern "*XBOX360_DOCUMENT_BUTTON_BACK*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_DOCUMENT_BUTTON_BACK* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_DOCUMENT_BUTTON_BACK* 6)
(intern "*XBOX360_LEFT_THUMB_BUTTON*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_LEFT_THUMB_BUTTON* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_LEFT_THUMB_BUTTON* 7)
(intern "*XBOX360_RIGHT_THUMB_BUTTON*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_RIGHT_THUMB_BUTTON* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_RIGHT_THUMB_BUTTON* 8)
(intern "*XBOX360_LEFT_SHOULDER*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_LEFT_SHOULDER* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_LEFT_SHOULDER* 9)
(intern "*XBOX360_RIGHT_SHOULDER*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_RIGHT_SHOULDER* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_RIGHT_SHOULDER* 10)
(intern "*XBOX360_BUTTON_A*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_BUTTON_A* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_A* 13)
(intern "*XBOX360_BUTTON_B*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_BUTTON_B* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_B* 14)
(intern "*XBOX360_BUTTON_X*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_BUTTON_X* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_X* 15)
(intern "*XBOX360_BUTTON_Y*" (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(shadow '*XBOX360_BUTTON_Y* (find-package "KORTEX_DRIVER::XBOX360DIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_Y* 16)

(defun kortex_driver::Xbox360DigitalInputIdentifier-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Xbox360DigitalInputIdentifier::*UNSPECIFIED_XBOX360_DIGITAL*)
        ((= const 1) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_UP*)
        ((= const 2) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_DOWN*)
        ((= const 3) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_LEFT*)
        ((= const 4) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_PAD_RIGHT*)
        ((= const 5) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_FILE_BUTTON_START*)
        ((= const 6) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_DOCUMENT_BUTTON_BACK*)
        ((= const 7) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_LEFT_THUMB_BUTTON*)
        ((= const 8) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_RIGHT_THUMB_BUTTON*)
        ((= const 9) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_LEFT_SHOULDER*)
        ((= const 10) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_RIGHT_SHOULDER*)
        ((= const 13) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_A*)
        ((= const 14) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_B*)
        ((= const 15) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_X*)
        ((= const 16) 'kortex_driver::Xbox360DigitalInputIdentifier::*XBOX360_BUTTON_Y*)
        (t nil)))

(defclass kortex_driver::Xbox360DigitalInputIdentifier
  :super ros::object
  :slots ())

(defmethod kortex_driver::Xbox360DigitalInputIdentifier
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

(setf (get kortex_driver::Xbox360DigitalInputIdentifier :md5sum-) "b1197219b8c5e86e78fa24aefadcb86e")
(setf (get kortex_driver::Xbox360DigitalInputIdentifier :datatype-) "kortex_driver/Xbox360DigitalInputIdentifier")
(setf (get kortex_driver::Xbox360DigitalInputIdentifier :definition-)
      "
uint32 UNSPECIFIED_XBOX360_DIGITAL = 0

uint32 XBOX360_PAD_UP = 1

uint32 XBOX360_PAD_DOWN = 2

uint32 XBOX360_PAD_LEFT = 3

uint32 XBOX360_PAD_RIGHT = 4

uint32 XBOX360_FILE_BUTTON_START = 5

uint32 XBOX360_DOCUMENT_BUTTON_BACK = 6

uint32 XBOX360_LEFT_THUMB_BUTTON = 7

uint32 XBOX360_RIGHT_THUMB_BUTTON = 8

uint32 XBOX360_LEFT_SHOULDER = 9

uint32 XBOX360_RIGHT_SHOULDER = 10

uint32 XBOX360_BUTTON_A = 13

uint32 XBOX360_BUTTON_B = 14

uint32 XBOX360_BUTTON_X = 15

uint32 XBOX360_BUTTON_Y = 16

")



(provide :kortex_driver/Xbox360DigitalInputIdentifier "b1197219b8c5e86e78fa24aefadcb86e")


