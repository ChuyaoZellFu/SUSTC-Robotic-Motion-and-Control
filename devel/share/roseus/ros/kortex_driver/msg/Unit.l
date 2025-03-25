;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Unit)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Unit (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UNIT")
  (make-package "KORTEX_DRIVER::UNIT"))

(in-package "ROS")
;;//! \htmlinclude Unit.msg.html


(intern "*UNSPECIFIED_UNIT*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*UNSPECIFIED_UNIT* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*UNSPECIFIED_UNIT* 0)
(intern "*CELSIUS*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*CELSIUS* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*CELSIUS* 1)
(intern "*AMPERE*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*AMPERE* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*AMPERE* 2)
(intern "*VOLT*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*VOLT* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*VOLT* 3)
(intern "*METER_PER_SECOND*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*METER_PER_SECOND* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*METER_PER_SECOND* 4)
(intern "*DEGREE_PER_SECOND*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*DEGREE_PER_SECOND* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*DEGREE_PER_SECOND* 5)
(intern "*METER_PER_SECOND_2*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*METER_PER_SECOND_2* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*METER_PER_SECOND_2* 6)
(intern "*DEGREE_PER_SECOND_2*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*DEGREE_PER_SECOND_2* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*DEGREE_PER_SECOND_2* 7)
(intern "*NEWTON*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*NEWTON* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*NEWTON* 8)
(intern "*NEWTON_METER*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*NEWTON_METER* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*NEWTON_METER* 9)
(intern "*KILOGRAM*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*KILOGRAM* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*KILOGRAM* 10)
(intern "*DEGREE*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*DEGREE* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*DEGREE* 11)
(intern "*TICK*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*TICK* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*TICK* 12)
(intern "*DEGREE_PER_MILLISECOND*" (find-package "KORTEX_DRIVER::UNIT"))
(shadow '*DEGREE_PER_MILLISECOND* (find-package "KORTEX_DRIVER::UNIT"))
(defconstant kortex_driver::Unit::*DEGREE_PER_MILLISECOND* 13)

(defun kortex_driver::Unit-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Unit::*UNSPECIFIED_UNIT*)
        ((= const 1) 'kortex_driver::Unit::*CELSIUS*)
        ((= const 2) 'kortex_driver::Unit::*AMPERE*)
        ((= const 3) 'kortex_driver::Unit::*VOLT*)
        ((= const 4) 'kortex_driver::Unit::*METER_PER_SECOND*)
        ((= const 5) 'kortex_driver::Unit::*DEGREE_PER_SECOND*)
        ((= const 6) 'kortex_driver::Unit::*METER_PER_SECOND_2*)
        ((= const 7) 'kortex_driver::Unit::*DEGREE_PER_SECOND_2*)
        ((= const 8) 'kortex_driver::Unit::*NEWTON*)
        ((= const 9) 'kortex_driver::Unit::*NEWTON_METER*)
        ((= const 10) 'kortex_driver::Unit::*KILOGRAM*)
        ((= const 11) 'kortex_driver::Unit::*DEGREE*)
        ((= const 12) 'kortex_driver::Unit::*TICK*)
        ((= const 13) 'kortex_driver::Unit::*DEGREE_PER_MILLISECOND*)
        (t nil)))

(defclass kortex_driver::Unit
  :super ros::object
  :slots ())

(defmethod kortex_driver::Unit
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

(setf (get kortex_driver::Unit :md5sum-) "38eeb5ea9959fe1af60e856b88a5f9a7")
(setf (get kortex_driver::Unit :datatype-) "kortex_driver/Unit")
(setf (get kortex_driver::Unit :definition-)
      "
uint32 UNSPECIFIED_UNIT = 0

uint32 CELSIUS = 1

uint32 AMPERE = 2

uint32 VOLT = 3

uint32 METER_PER_SECOND = 4

uint32 DEGREE_PER_SECOND = 5

uint32 METER_PER_SECOND_2 = 6

uint32 DEGREE_PER_SECOND_2 = 7

uint32 NEWTON = 8

uint32 NEWTON_METER = 9

uint32 KILOGRAM = 10

uint32 DEGREE = 11

uint32 TICK = 12

uint32 DEGREE_PER_MILLISECOND = 13

")



(provide :kortex_driver/Unit "38eeb5ea9959fe1af60e856b88a5f9a7")


