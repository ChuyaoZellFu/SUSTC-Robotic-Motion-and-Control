;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CReadRegisterParameter)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CReadRegisterParameter (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CREADREGISTERPARAMETER")
  (make-package "KORTEX_DRIVER::I2CREADREGISTERPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude I2CReadRegisterParameter.msg.html


(defclass kortex_driver::I2CReadRegisterParameter
  :super ros::object
  :slots (_device _device_address _register_address _register_address_size _size _timeout ))

(defmethod kortex_driver::I2CReadRegisterParameter
  (:init
   (&key
    ((:device __device) 0)
    ((:device_address __device_address) 0)
    ((:register_address __register_address) 0)
    ((:register_address_size __register_address_size) 0)
    ((:size __size) 0)
    ((:timeout __timeout) 0)
    )
   (send-super :init)
   (setq _device (round __device))
   (setq _device_address (round __device_address))
   (setq _register_address (round __register_address))
   (setq _register_address_size (round __register_address_size))
   (setq _size (round __size))
   (setq _timeout (round __timeout))
   self)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:device_address
   (&optional __device_address)
   (if __device_address (setq _device_address __device_address)) _device_address)
  (:register_address
   (&optional __register_address)
   (if __register_address (setq _register_address __register_address)) _register_address)
  (:register_address_size
   (&optional __register_address_size)
   (if __register_address_size (setq _register_address_size __register_address_size)) _register_address_size)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:serialization-length
   ()
   (+
    ;; uint32 _device
    4
    ;; uint32 _device_address
    4
    ;; uint32 _register_address
    4
    ;; uint32 _register_address_size
    4
    ;; uint32 _size
    4
    ;; uint32 _timeout
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device
       (write-long _device s)
     ;; uint32 _device_address
       (write-long _device_address s)
     ;; uint32 _register_address
       (write-long _register_address s)
     ;; uint32 _register_address_size
       (write-long _register_address_size s)
     ;; uint32 _size
       (write-long _size s)
     ;; uint32 _timeout
       (write-long _timeout s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _device_address
     (setq _device_address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _register_address
     (setq _register_address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _register_address_size
     (setq _register_address_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _timeout
     (setq _timeout (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::I2CReadRegisterParameter :md5sum-) "5d3209e38cc377eccf27593ef8027f34")
(setf (get kortex_driver::I2CReadRegisterParameter :datatype-) "kortex_driver/I2CReadRegisterParameter")
(setf (get kortex_driver::I2CReadRegisterParameter :definition-)
      "
uint32 device
uint32 device_address
uint32 register_address
uint32 register_address_size
uint32 size
uint32 timeout
")



(provide :kortex_driver/I2CReadRegisterParameter "5d3209e38cc377eccf27593ef8027f34")


