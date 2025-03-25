;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::LimitationType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'LimitationType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::LIMITATIONTYPE")
  (make-package "KORTEX_DRIVER::LIMITATIONTYPE"))

(in-package "ROS")
;;//! \htmlinclude LimitationType.msg.html


(intern "*UNSPECIFIED_LIMITATION*" (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(shadow '*UNSPECIFIED_LIMITATION* (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(defconstant kortex_driver::LimitationType::*UNSPECIFIED_LIMITATION* 0)
(intern "*FORCE_LIMITATION*" (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(shadow '*FORCE_LIMITATION* (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(defconstant kortex_driver::LimitationType::*FORCE_LIMITATION* 1)
(intern "*ACCELERATION_LIMITATION*" (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(shadow '*ACCELERATION_LIMITATION* (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(defconstant kortex_driver::LimitationType::*ACCELERATION_LIMITATION* 2)
(intern "*VELOCITY_LIMITATION*" (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(shadow '*VELOCITY_LIMITATION* (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(defconstant kortex_driver::LimitationType::*VELOCITY_LIMITATION* 3)
(intern "*TORQUE_LIMITATION*" (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(shadow '*TORQUE_LIMITATION* (find-package "KORTEX_DRIVER::LIMITATIONTYPE"))
(defconstant kortex_driver::LimitationType::*TORQUE_LIMITATION* 4)

(defun kortex_driver::LimitationType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::LimitationType::*UNSPECIFIED_LIMITATION*)
        ((= const 1) 'kortex_driver::LimitationType::*FORCE_LIMITATION*)
        ((= const 2) 'kortex_driver::LimitationType::*ACCELERATION_LIMITATION*)
        ((= const 3) 'kortex_driver::LimitationType::*VELOCITY_LIMITATION*)
        ((= const 4) 'kortex_driver::LimitationType::*TORQUE_LIMITATION*)
        (t nil)))

(defclass kortex_driver::LimitationType
  :super ros::object
  :slots ())

(defmethod kortex_driver::LimitationType
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

(setf (get kortex_driver::LimitationType :md5sum-) "86bbe126d2c218724be7709cc366b9d5")
(setf (get kortex_driver::LimitationType :datatype-) "kortex_driver/LimitationType")
(setf (get kortex_driver::LimitationType :definition-)
      "
uint32 UNSPECIFIED_LIMITATION = 0

uint32 FORCE_LIMITATION = 1

uint32 ACCELERATION_LIMITATION = 2

uint32 VELOCITY_LIMITATION = 3

uint32 TORQUE_LIMITATION = 4

")



(provide :kortex_driver/LimitationType "86bbe126d2c218724be7709cc366b9d5")


