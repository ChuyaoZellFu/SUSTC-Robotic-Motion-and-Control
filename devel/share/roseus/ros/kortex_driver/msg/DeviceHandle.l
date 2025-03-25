;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeviceHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeviceHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DEVICEHANDLE")
  (make-package "KORTEX_DRIVER::DEVICEHANDLE"))

(in-package "ROS")
;;//! \htmlinclude DeviceHandle.msg.html


(defclass kortex_driver::DeviceHandle
  :super ros::object
  :slots (_device_type _device_identifier _order ))

(defmethod kortex_driver::DeviceHandle
  (:init
   (&key
    ((:device_type __device_type) 0)
    ((:device_identifier __device_identifier) 0)
    ((:order __order) 0)
    )
   (send-super :init)
   (setq _device_type (round __device_type))
   (setq _device_identifier (round __device_identifier))
   (setq _order (round __order))
   self)
  (:device_type
   (&optional __device_type)
   (if __device_type (setq _device_type __device_type)) _device_type)
  (:device_identifier
   (&optional __device_identifier)
   (if __device_identifier (setq _device_identifier __device_identifier)) _device_identifier)
  (:order
   (&optional __order)
   (if __order (setq _order __order)) _order)
  (:serialization-length
   ()
   (+
    ;; uint32 _device_type
    4
    ;; uint32 _device_identifier
    4
    ;; uint32 _order
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device_type
       (write-long _device_type s)
     ;; uint32 _device_identifier
       (write-long _device_identifier s)
     ;; uint32 _order
       (write-long _order s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device_type
     (setq _device_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _device_identifier
     (setq _device_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _order
     (setq _order (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::DeviceHandle :md5sum-) "a89d226be38807777119fd9acd7acb3b")
(setf (get kortex_driver::DeviceHandle :datatype-) "kortex_driver/DeviceHandle")
(setf (get kortex_driver::DeviceHandle :definition-)
      "
uint32 device_type
uint32 device_identifier
uint32 order
")



(provide :kortex_driver/DeviceHandle "a89d226be38807777119fd9acd7acb3b")


