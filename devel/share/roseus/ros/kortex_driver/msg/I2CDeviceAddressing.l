;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CDeviceAddressing)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CDeviceAddressing (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CDEVICEADDRESSING")
  (make-package "KORTEX_DRIVER::I2CDEVICEADDRESSING"))

(in-package "ROS")
;;//! \htmlinclude I2CDeviceAddressing.msg.html


(intern "*I2C_DEVICE_ADDRESSING_UNSPECIFIED*" (find-package "KORTEX_DRIVER::I2CDEVICEADDRESSING"))
(shadow '*I2C_DEVICE_ADDRESSING_UNSPECIFIED* (find-package "KORTEX_DRIVER::I2CDEVICEADDRESSING"))
(defconstant kortex_driver::I2CDeviceAddressing::*I2C_DEVICE_ADDRESSING_UNSPECIFIED* 0)
(intern "*I2C_DEVICE_ADDRESSING_7_BITS*" (find-package "KORTEX_DRIVER::I2CDEVICEADDRESSING"))
(shadow '*I2C_DEVICE_ADDRESSING_7_BITS* (find-package "KORTEX_DRIVER::I2CDEVICEADDRESSING"))
(defconstant kortex_driver::I2CDeviceAddressing::*I2C_DEVICE_ADDRESSING_7_BITS* 1)
(intern "*I2C_DEVICE_ADDRESSING_10_BITS*" (find-package "KORTEX_DRIVER::I2CDEVICEADDRESSING"))
(shadow '*I2C_DEVICE_ADDRESSING_10_BITS* (find-package "KORTEX_DRIVER::I2CDEVICEADDRESSING"))
(defconstant kortex_driver::I2CDeviceAddressing::*I2C_DEVICE_ADDRESSING_10_BITS* 2)

(defun kortex_driver::I2CDeviceAddressing-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::I2CDeviceAddressing::*I2C_DEVICE_ADDRESSING_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::I2CDeviceAddressing::*I2C_DEVICE_ADDRESSING_7_BITS*)
        ((= const 2) 'kortex_driver::I2CDeviceAddressing::*I2C_DEVICE_ADDRESSING_10_BITS*)
        (t nil)))

(defclass kortex_driver::I2CDeviceAddressing
  :super ros::object
  :slots ())

(defmethod kortex_driver::I2CDeviceAddressing
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

(setf (get kortex_driver::I2CDeviceAddressing :md5sum-) "a6df7de78b1ed95513b026e1947c58a0")
(setf (get kortex_driver::I2CDeviceAddressing :datatype-) "kortex_driver/I2CDeviceAddressing")
(setf (get kortex_driver::I2CDeviceAddressing :definition-)
      "
uint32 I2C_DEVICE_ADDRESSING_UNSPECIFIED = 0

uint32 I2C_DEVICE_ADDRESSING_7_BITS = 1

uint32 I2C_DEVICE_ADDRESSING_10_BITS = 2

")



(provide :kortex_driver/I2CDeviceAddressing "a6df7de78b1ed95513b026e1947c58a0")


