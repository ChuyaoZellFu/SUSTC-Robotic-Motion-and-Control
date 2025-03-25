;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EthernetDeviceIdentification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EthernetDeviceIdentification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ETHERNETDEVICEIDENTIFICATION")
  (make-package "KORTEX_DRIVER::ETHERNETDEVICEIDENTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude EthernetDeviceIdentification.msg.html


(defclass kortex_driver::EthernetDeviceIdentification
  :super ros::object
  :slots (_device ))

(defmethod kortex_driver::EthernetDeviceIdentification
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

(setf (get kortex_driver::EthernetDeviceIdentification :md5sum-) "e97054e4ab2eb3665270f077c46482df")
(setf (get kortex_driver::EthernetDeviceIdentification :datatype-) "kortex_driver/EthernetDeviceIdentification")
(setf (get kortex_driver::EthernetDeviceIdentification :definition-)
      "
uint32 device
")



(provide :kortex_driver/EthernetDeviceIdentification "e97054e4ab2eb3665270f077c46482df")


