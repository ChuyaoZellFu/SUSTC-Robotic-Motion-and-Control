;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CWriteParameter)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CWriteParameter (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CWRITEPARAMETER")
  (make-package "KORTEX_DRIVER::I2CWRITEPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude I2CWriteParameter.msg.html


(defclass kortex_driver::I2CWriteParameter
  :super ros::object
  :slots (_device _device_address _timeout _data ))

(defmethod kortex_driver::I2CWriteParameter
  (:init
   (&key
    ((:device __device) 0)
    ((:device_address __device_address) 0)
    ((:timeout __timeout) 0)
    ((:data __data) (instance kortex_driver::I2CData :init))
    )
   (send-super :init)
   (setq _device (round __device))
   (setq _device_address (round __device_address))
   (setq _timeout (round __timeout))
   (setq _data __data)
   self)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:device_address
   (&optional __device_address)
   (if __device_address (setq _device_address __device_address)) _device_address)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; uint32 _device
    4
    ;; uint32 _device_address
    4
    ;; uint32 _timeout
    4
    ;; kortex_driver/I2CData _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device
       (write-long _device s)
     ;; uint32 _device_address
       (write-long _device_address s)
     ;; uint32 _timeout
       (write-long _timeout s)
     ;; kortex_driver/I2CData _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _device_address
     (setq _device_address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _timeout
     (setq _timeout (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/I2CData _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::I2CWriteParameter :md5sum-) "619677bad5497ce3141f386d2779fe02")
(setf (get kortex_driver::I2CWriteParameter :datatype-) "kortex_driver/I2CWriteParameter")
(setf (get kortex_driver::I2CWriteParameter :definition-)
      "
uint32 device
uint32 device_address
uint32 timeout
I2CData data
================================================================================
MSG: kortex_driver/I2CData

uint8[] data
uint32 size
")



(provide :kortex_driver/I2CWriteParameter "619677bad5497ce3141f386d2779fe02")


