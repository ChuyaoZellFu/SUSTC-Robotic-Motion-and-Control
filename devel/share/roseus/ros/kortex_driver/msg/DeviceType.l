;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeviceType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeviceType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DEVICETYPE")
  (make-package "KORTEX_DRIVER::DEVICETYPE"))

(in-package "ROS")
;;//! \htmlinclude DeviceType.msg.html


(defclass kortex_driver::DeviceType
  :super ros::object
  :slots (_device_type ))

(defmethod kortex_driver::DeviceType
  (:init
   (&key
    ((:device_type __device_type) 0)
    )
   (send-super :init)
   (setq _device_type (round __device_type))
   self)
  (:device_type
   (&optional __device_type)
   (if __device_type (setq _device_type __device_type)) _device_type)
  (:serialization-length
   ()
   (+
    ;; uint32 _device_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device_type
       (write-long _device_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device_type
     (setq _device_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::DeviceType :md5sum-) "03c69ac8f4cbd0c35678776a27f8d84a")
(setf (get kortex_driver::DeviceType :datatype-) "kortex_driver/DeviceType")
(setf (get kortex_driver::DeviceType :definition-)
      "
uint32 device_type
")



(provide :kortex_driver/DeviceType "03c69ac8f4cbd0c35678776a27f8d84a")


