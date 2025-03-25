;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ServoingMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ServoingMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SERVOINGMODE")
  (make-package "KORTEX_DRIVER::SERVOINGMODE"))

(in-package "ROS")
;;//! \htmlinclude ServoingMode.msg.html


(intern "*UNSPECIFIED_SERVOING_MODE*" (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(shadow '*UNSPECIFIED_SERVOING_MODE* (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(defconstant kortex_driver::ServoingMode::*UNSPECIFIED_SERVOING_MODE* 0)
(intern "*SINGLE_LEVEL_SERVOING*" (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(shadow '*SINGLE_LEVEL_SERVOING* (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(defconstant kortex_driver::ServoingMode::*SINGLE_LEVEL_SERVOING* 2)
(intern "*LOW_LEVEL_SERVOING*" (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(shadow '*LOW_LEVEL_SERVOING* (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(defconstant kortex_driver::ServoingMode::*LOW_LEVEL_SERVOING* 3)
(intern "*BYPASS_SERVOING*" (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(shadow '*BYPASS_SERVOING* (find-package "KORTEX_DRIVER::SERVOINGMODE"))
(defconstant kortex_driver::ServoingMode::*BYPASS_SERVOING* 4)

(defun kortex_driver::ServoingMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ServoingMode::*UNSPECIFIED_SERVOING_MODE*)
        ((= const 2) 'kortex_driver::ServoingMode::*SINGLE_LEVEL_SERVOING*)
        ((= const 3) 'kortex_driver::ServoingMode::*LOW_LEVEL_SERVOING*)
        ((= const 4) 'kortex_driver::ServoingMode::*BYPASS_SERVOING*)
        (t nil)))

(defclass kortex_driver::ServoingMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::ServoingMode
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

(setf (get kortex_driver::ServoingMode :md5sum-) "3ce819b6b24c3de6343a9b4a6de953eb")
(setf (get kortex_driver::ServoingMode :datatype-) "kortex_driver/ServoingMode")
(setf (get kortex_driver::ServoingMode :definition-)
      "
uint32 UNSPECIFIED_SERVOING_MODE = 0

uint32 SINGLE_LEVEL_SERVOING = 2

uint32 LOW_LEVEL_SERVOING = 3

uint32 BYPASS_SERVOING = 4

")



(provide :kortex_driver/ServoingMode "3ce819b6b24c3de6343a9b4a6de953eb")


