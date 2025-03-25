;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StatusFlags)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StatusFlags (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STATUSFLAGS")
  (make-package "KORTEX_DRIVER::STATUSFLAGS"))

(in-package "ROS")
;;//! \htmlinclude StatusFlags.msg.html


(intern "*UNKNOWN_STATUS*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*UNKNOWN_STATUS* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*UNKNOWN_STATUS* 0)
(intern "*STABILIZED*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*STABILIZED* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*STABILIZED* 16)
(intern "*MOTOR_INDEXED*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*MOTOR_INDEXED* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*MOTOR_INDEXED* 32)
(intern "*MOTOR_INDEXING*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*MOTOR_INDEXING* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*MOTOR_INDEXING* 64)
(intern "*JOINT_INDEXED*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*JOINT_INDEXED* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*JOINT_INDEXED* 128)
(intern "*JOINT_INDEXING*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*JOINT_INDEXING* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*JOINT_INDEXING* 256)
(intern "*HIGH_PRECISION*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*HIGH_PRECISION* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*HIGH_PRECISION* 512)
(intern "*BRAKING*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*BRAKING* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*BRAKING* 1024)
(intern "*SERVOING*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*SERVOING* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*SERVOING* 2048)
(intern "*MAJOR_FAULT*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*MAJOR_FAULT* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*MAJOR_FAULT* 4096)
(intern "*MINOR_FAULT*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*MINOR_FAULT* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*MINOR_FAULT* 8192)
(intern "*CALIBRATED_TORQUE*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*CALIBRATED_TORQUE* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*CALIBRATED_TORQUE* 16384)
(intern "*CALIBRATED_MAG_SENSOR*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*CALIBRATED_MAG_SENSOR* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*CALIBRATED_MAG_SENSOR* 32768)
(intern "*CALIBRATED_ZERO*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*CALIBRATED_ZERO* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*CALIBRATED_ZERO* 65536)
(intern "*GPIO_0*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*GPIO_0* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*GPIO_0* 131072)
(intern "*GPIO_1*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*GPIO_1* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*GPIO_1* 262144)
(intern "*CS_QUASI_STATIC_CONTACT*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*CS_QUASI_STATIC_CONTACT* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*CS_QUASI_STATIC_CONTACT* 524288)
(intern "*CS_TRANSIENT_CONTACT*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*CS_TRANSIENT_CONTACT* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*CS_TRANSIENT_CONTACT* 1048576)
(intern "*VFD_HALL_SYNC*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*VFD_HALL_SYNC* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*VFD_HALL_SYNC* 2097152)
(intern "*VFD_INDEXED*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*VFD_INDEXED* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*VFD_INDEXED* 4194304)
(intern "*DRIVE_BOARD_READY*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*DRIVE_BOARD_READY* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*DRIVE_BOARD_READY* 8388608)
(intern "*CALIBRATED_CURRENT*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*CALIBRATED_CURRENT* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*CALIBRATED_CURRENT* 16777216)
(intern "*CALIBRATED_MOTOR*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*CALIBRATED_MOTOR* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*CALIBRATED_MOTOR* 33554432)
(intern "*SW0_ACTIVE*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*SW0_ACTIVE* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*SW0_ACTIVE* 67108864)
(intern "*SW1_ACTIVE*" (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(shadow '*SW1_ACTIVE* (find-package "KORTEX_DRIVER::STATUSFLAGS"))
(defconstant kortex_driver::StatusFlags::*SW1_ACTIVE* 134217728)

(defun kortex_driver::StatusFlags-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::StatusFlags::*UNKNOWN_STATUS*)
        ((= const 16) 'kortex_driver::StatusFlags::*STABILIZED*)
        ((= const 32) 'kortex_driver::StatusFlags::*MOTOR_INDEXED*)
        ((= const 64) 'kortex_driver::StatusFlags::*MOTOR_INDEXING*)
        ((= const 128) 'kortex_driver::StatusFlags::*JOINT_INDEXED*)
        ((= const 256) 'kortex_driver::StatusFlags::*JOINT_INDEXING*)
        ((= const 512) 'kortex_driver::StatusFlags::*HIGH_PRECISION*)
        ((= const 1024) 'kortex_driver::StatusFlags::*BRAKING*)
        ((= const 2048) 'kortex_driver::StatusFlags::*SERVOING*)
        ((= const 4096) 'kortex_driver::StatusFlags::*MAJOR_FAULT*)
        ((= const 8192) 'kortex_driver::StatusFlags::*MINOR_FAULT*)
        ((= const 16384) 'kortex_driver::StatusFlags::*CALIBRATED_TORQUE*)
        ((= const 32768) 'kortex_driver::StatusFlags::*CALIBRATED_MAG_SENSOR*)
        ((= const 65536) 'kortex_driver::StatusFlags::*CALIBRATED_ZERO*)
        ((= const 131072) 'kortex_driver::StatusFlags::*GPIO_0*)
        ((= const 262144) 'kortex_driver::StatusFlags::*GPIO_1*)
        ((= const 524288) 'kortex_driver::StatusFlags::*CS_QUASI_STATIC_CONTACT*)
        ((= const 1048576) 'kortex_driver::StatusFlags::*CS_TRANSIENT_CONTACT*)
        ((= const 2097152) 'kortex_driver::StatusFlags::*VFD_HALL_SYNC*)
        ((= const 4194304) 'kortex_driver::StatusFlags::*VFD_INDEXED*)
        ((= const 8388608) 'kortex_driver::StatusFlags::*DRIVE_BOARD_READY*)
        ((= const 16777216) 'kortex_driver::StatusFlags::*CALIBRATED_CURRENT*)
        ((= const 33554432) 'kortex_driver::StatusFlags::*CALIBRATED_MOTOR*)
        ((= const 67108864) 'kortex_driver::StatusFlags::*SW0_ACTIVE*)
        ((= const 134217728) 'kortex_driver::StatusFlags::*SW1_ACTIVE*)
        (t nil)))

(defclass kortex_driver::StatusFlags
  :super ros::object
  :slots ())

(defmethod kortex_driver::StatusFlags
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

(setf (get kortex_driver::StatusFlags :md5sum-) "94f8e368a8b96139de6828bbd6ed2797")
(setf (get kortex_driver::StatusFlags :datatype-) "kortex_driver/StatusFlags")
(setf (get kortex_driver::StatusFlags :definition-)
      "
uint32 UNKNOWN_STATUS = 0

uint32 STABILIZED = 16

uint32 MOTOR_INDEXED = 32

uint32 MOTOR_INDEXING = 64

uint32 JOINT_INDEXED = 128

uint32 JOINT_INDEXING = 256

uint32 HIGH_PRECISION = 512

uint32 BRAKING = 1024

uint32 SERVOING = 2048

uint32 MAJOR_FAULT = 4096

uint32 MINOR_FAULT = 8192

uint32 CALIBRATED_TORQUE = 16384

uint32 CALIBRATED_MAG_SENSOR = 32768

uint32 CALIBRATED_ZERO = 65536

uint32 GPIO_0 = 131072

uint32 GPIO_1 = 262144

uint32 CS_QUASI_STATIC_CONTACT = 524288

uint32 CS_TRANSIENT_CONTACT = 1048576

uint32 VFD_HALL_SYNC = 2097152

uint32 VFD_INDEXED = 4194304

uint32 DRIVE_BOARD_READY = 8388608

uint32 CALIBRATED_CURRENT = 16777216

uint32 CALIBRATED_MOTOR = 33554432

uint32 SW0_ACTIVE = 67108864

uint32 SW1_ACTIVE = 134217728

")



(provide :kortex_driver/StatusFlags "94f8e368a8b96139de6828bbd6ed2797")


