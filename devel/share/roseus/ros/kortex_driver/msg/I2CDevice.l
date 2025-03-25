;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CDevice)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CDevice (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CDEVICE")
  (make-package "KORTEX_DRIVER::I2CDEVICE"))

(in-package "ROS")
;;//! \htmlinclude I2CDevice.msg.html


(intern "*I2C_DEVICE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::I2CDEVICE"))
(shadow '*I2C_DEVICE_UNSPECIFIED* (find-package "KORTEX_DRIVER::I2CDEVICE"))
(defconstant kortex_driver::I2CDevice::*I2C_DEVICE_UNSPECIFIED* 0)
(intern "*I2C_DEVICE_EXPANSION*" (find-package "KORTEX_DRIVER::I2CDEVICE"))
(shadow '*I2C_DEVICE_EXPANSION* (find-package "KORTEX_DRIVER::I2CDEVICE"))
(defconstant kortex_driver::I2CDevice::*I2C_DEVICE_EXPANSION* 1)

(defun kortex_driver::I2CDevice-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::I2CDevice::*I2C_DEVICE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::I2CDevice::*I2C_DEVICE_EXPANSION*)
        (t nil)))

(defclass kortex_driver::I2CDevice
  :super ros::object
  :slots ())

(defmethod kortex_driver::I2CDevice
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

(setf (get kortex_driver::I2CDevice :md5sum-) "f989991b5a55a050960f41f9ed3a70ea")
(setf (get kortex_driver::I2CDevice :datatype-) "kortex_driver/I2CDevice")
(setf (get kortex_driver::I2CDevice :definition-)
      "
uint32 I2C_DEVICE_UNSPECIFIED = 0

uint32 I2C_DEVICE_EXPANSION = 1

")



(provide :kortex_driver/I2CDevice "f989991b5a55a050960f41f9ed3a70ea")


