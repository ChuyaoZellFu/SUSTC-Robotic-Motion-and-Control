;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WristDigitalInputIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WristDigitalInputIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER")
  (make-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude WristDigitalInputIdentifier.msg.html


(intern "*UNSPECIFIED_WRIST_DIGITAL*" (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(shadow '*UNSPECIFIED_WRIST_DIGITAL* (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::WristDigitalInputIdentifier::*UNSPECIFIED_WRIST_DIGITAL* 0)
(intern "*WRIST_BUTTON_1*" (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(shadow '*WRIST_BUTTON_1* (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::WristDigitalInputIdentifier::*WRIST_BUTTON_1* 1)
(intern "*WRIST_BUTTON_2*" (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(shadow '*WRIST_BUTTON_2* (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::WristDigitalInputIdentifier::*WRIST_BUTTON_2* 2)
(intern "*WRIST_BUTTON_BOTH*" (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(shadow '*WRIST_BUTTON_BOTH* (find-package "KORTEX_DRIVER::WRISTDIGITALINPUTIDENTIFIER"))
(defconstant kortex_driver::WristDigitalInputIdentifier::*WRIST_BUTTON_BOTH* 3)

(defun kortex_driver::WristDigitalInputIdentifier-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::WristDigitalInputIdentifier::*UNSPECIFIED_WRIST_DIGITAL*)
        ((= const 1) 'kortex_driver::WristDigitalInputIdentifier::*WRIST_BUTTON_1*)
        ((= const 2) 'kortex_driver::WristDigitalInputIdentifier::*WRIST_BUTTON_2*)
        ((= const 3) 'kortex_driver::WristDigitalInputIdentifier::*WRIST_BUTTON_BOTH*)
        (t nil)))

(defclass kortex_driver::WristDigitalInputIdentifier
  :super ros::object
  :slots ())

(defmethod kortex_driver::WristDigitalInputIdentifier
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

(setf (get kortex_driver::WristDigitalInputIdentifier :md5sum-) "1ca46a2da6d637ba3dfa3b30f7d69da3")
(setf (get kortex_driver::WristDigitalInputIdentifier :datatype-) "kortex_driver/WristDigitalInputIdentifier")
(setf (get kortex_driver::WristDigitalInputIdentifier :definition-)
      "
uint32 UNSPECIFIED_WRIST_DIGITAL = 0

uint32 WRIST_BUTTON_1 = 1

uint32 WRIST_BUTTON_2 = 2

uint32 WRIST_BUTTON_BOTH = 3

")



(provide :kortex_driver/WristDigitalInputIdentifier "1ca46a2da6d637ba3dfa3b30f7d69da3")


