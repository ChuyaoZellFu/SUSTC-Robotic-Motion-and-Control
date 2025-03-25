;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CRegisterAddressSize)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CRegisterAddressSize (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE")
  (make-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE"))

(in-package "ROS")
;;//! \htmlinclude I2CRegisterAddressSize.msg.html


(intern "*I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE"))
(shadow '*I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED* (find-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE"))
(defconstant kortex_driver::I2CRegisterAddressSize::*I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED* 0)
(intern "*I2C_REGISTER_ADDRESS_SIZE_8_BITS*" (find-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE"))
(shadow '*I2C_REGISTER_ADDRESS_SIZE_8_BITS* (find-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE"))
(defconstant kortex_driver::I2CRegisterAddressSize::*I2C_REGISTER_ADDRESS_SIZE_8_BITS* 1)
(intern "*I2C_REGISTER_ADDRESS_SIZE_16_BITS*" (find-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE"))
(shadow '*I2C_REGISTER_ADDRESS_SIZE_16_BITS* (find-package "KORTEX_DRIVER::I2CREGISTERADDRESSSIZE"))
(defconstant kortex_driver::I2CRegisterAddressSize::*I2C_REGISTER_ADDRESS_SIZE_16_BITS* 2)

(defun kortex_driver::I2CRegisterAddressSize-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::I2CRegisterAddressSize::*I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::I2CRegisterAddressSize::*I2C_REGISTER_ADDRESS_SIZE_8_BITS*)
        ((= const 2) 'kortex_driver::I2CRegisterAddressSize::*I2C_REGISTER_ADDRESS_SIZE_16_BITS*)
        (t nil)))

(defclass kortex_driver::I2CRegisterAddressSize
  :super ros::object
  :slots ())

(defmethod kortex_driver::I2CRegisterAddressSize
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

(setf (get kortex_driver::I2CRegisterAddressSize :md5sum-) "517dd96a7b250066663891b1fad6bd8a")
(setf (get kortex_driver::I2CRegisterAddressSize :datatype-) "kortex_driver/I2CRegisterAddressSize")
(setf (get kortex_driver::I2CRegisterAddressSize :definition-)
      "
uint32 I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED = 0

uint32 I2C_REGISTER_ADDRESS_SIZE_8_BITS = 1

uint32 I2C_REGISTER_ADDRESS_SIZE_16_BITS = 2

")



(provide :kortex_driver/I2CRegisterAddressSize "517dd96a7b250066663891b1fad6bd8a")


