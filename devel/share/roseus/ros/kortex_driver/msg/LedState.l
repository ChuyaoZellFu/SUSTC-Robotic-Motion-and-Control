;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::LedState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'LedState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::LEDSTATE")
  (make-package "KORTEX_DRIVER::LEDSTATE"))

(in-package "ROS")
;;//! \htmlinclude LedState.msg.html


(intern "*UNSPECIFIED_LED_STATE*" (find-package "KORTEX_DRIVER::LEDSTATE"))
(shadow '*UNSPECIFIED_LED_STATE* (find-package "KORTEX_DRIVER::LEDSTATE"))
(defconstant kortex_driver::LedState::*UNSPECIFIED_LED_STATE* 0)
(intern "*LED_OFF*" (find-package "KORTEX_DRIVER::LEDSTATE"))
(shadow '*LED_OFF* (find-package "KORTEX_DRIVER::LEDSTATE"))
(defconstant kortex_driver::LedState::*LED_OFF* 1)
(intern "*LED_PULSE*" (find-package "KORTEX_DRIVER::LEDSTATE"))
(shadow '*LED_PULSE* (find-package "KORTEX_DRIVER::LEDSTATE"))
(defconstant kortex_driver::LedState::*LED_PULSE* 2)
(intern "*LED_ON*" (find-package "KORTEX_DRIVER::LEDSTATE"))
(shadow '*LED_ON* (find-package "KORTEX_DRIVER::LEDSTATE"))
(defconstant kortex_driver::LedState::*LED_ON* 3)

(defun kortex_driver::LedState-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::LedState::*UNSPECIFIED_LED_STATE*)
        ((= const 1) 'kortex_driver::LedState::*LED_OFF*)
        ((= const 2) 'kortex_driver::LedState::*LED_PULSE*)
        ((= const 3) 'kortex_driver::LedState::*LED_ON*)
        (t nil)))

(defclass kortex_driver::LedState
  :super ros::object
  :slots ())

(defmethod kortex_driver::LedState
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

(setf (get kortex_driver::LedState :md5sum-) "d10da8147549713ab2f5b5d7555fb6b6")
(setf (get kortex_driver::LedState :datatype-) "kortex_driver/LedState")
(setf (get kortex_driver::LedState :definition-)
      "
uint32 UNSPECIFIED_LED_STATE = 0

uint32 LED_OFF = 1

uint32 LED_PULSE = 2

uint32 LED_ON = 3

")



(provide :kortex_driver/LedState "d10da8147549713ab2f5b5d7555fb6b6")


