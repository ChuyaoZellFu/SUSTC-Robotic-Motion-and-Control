;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CReadParameter)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CReadParameter (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CREADPARAMETER")
  (make-package "KORTEX_DRIVER::I2CREADPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude I2CReadParameter.msg.html


(defclass kortex_driver::I2CReadParameter
  :super ros::object
  :slots (_device _device_address _size _timeout ))

(defmethod kortex_driver::I2CReadParameter
  (:init
   (&key
    ((:device __device) 0)
    ((:device_address __device_address) 0)
    ((:size __size) 0)
    ((:timeout __timeout) 0)
    )
   (send-super :init)
   (setq _device (round __device))
   (setq _device_address (round __device_address))
   (setq _size (round __size))
   (setq _timeout (round __timeout))
   self)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:device_address
   (&optional __device_address)
   (if __device_address (setq _device_address __device_address)) _device_address)
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
   ;; uint32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _timeout
     (setq _timeout (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::I2CReadParameter :md5sum-) "66188a52c1e1bb7e3a095189a7bded0c")
(setf (get kortex_driver::I2CReadParameter :datatype-) "kortex_driver/I2CReadParameter")
(setf (get kortex_driver::I2CReadParameter :definition-)
      "
uint32 device
uint32 device_address
uint32 size
uint32 timeout
")



(provide :kortex_driver/I2CReadParameter "66188a52c1e1bb7e3a095189a7bded0c")


