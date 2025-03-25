;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CDeviceIdentification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CDeviceIdentification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CDEVICEIDENTIFICATION")
  (make-package "KORTEX_DRIVER::I2CDEVICEIDENTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude I2CDeviceIdentification.msg.html


(defclass kortex_driver::I2CDeviceIdentification
  :super ros::object
  :slots (_device ))

(defmethod kortex_driver::I2CDeviceIdentification
  (:init
   (&key
    ((:device __device) 0)
    )
   (send-super :init)
   (setq _device (round __device))
   self)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:serialization-length
   ()
   (+
    ;; uint32 _device
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device
       (write-long _device s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::I2CDeviceIdentification :md5sum-) "e97054e4ab2eb3665270f077c46482df")
(setf (get kortex_driver::I2CDeviceIdentification :datatype-) "kortex_driver/I2CDeviceIdentification")
(setf (get kortex_driver::I2CDeviceIdentification :definition-)
      "
uint32 device
")



(provide :kortex_driver/I2CDeviceIdentification "e97054e4ab2eb3665270f077c46482df")


