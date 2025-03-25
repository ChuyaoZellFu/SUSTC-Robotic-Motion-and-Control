;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetDeviceID)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetDeviceID (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETDEVICEID")
  (make-package "KORTEX_DRIVER::SETDEVICEID"))
(unless (find-package "KORTEX_DRIVER::SETDEVICEIDREQUEST")
  (make-package "KORTEX_DRIVER::SETDEVICEIDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETDEVICEIDRESPONSE")
  (make-package "KORTEX_DRIVER::SETDEVICEIDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetDeviceIDRequest
  :super ros::object
  :slots (_device_id ))

(defmethod kortex_driver::SetDeviceIDRequest
  (:init
   (&key
    ((:device_id __device_id) 0)
    )
   (send-super :init)
   (setq _device_id (round __device_id))
   self)
  (:device_id
   (&optional __device_id)
   (if __device_id (setq _device_id __device_id)) _device_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _device_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device_id
       (write-long _device_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device_id
     (setq _device_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass kortex_driver::SetDeviceIDResponse
  :super ros::object
  :slots ())

(defmethod kortex_driver::SetDeviceIDResponse
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

(defclass kortex_driver::SetDeviceID
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetDeviceID :md5sum-) "983e371f5f8e03bba53608e89ded6b57")
(setf (get kortex_driver::SetDeviceID :datatype-) "kortex_driver/SetDeviceID")
(setf (get kortex_driver::SetDeviceID :request) kortex_driver::SetDeviceIDRequest)
(setf (get kortex_driver::SetDeviceID :response) kortex_driver::SetDeviceIDResponse)

(defmethod kortex_driver::SetDeviceIDRequest
  (:response () (instance kortex_driver::SetDeviceIDResponse :init)))

(setf (get kortex_driver::SetDeviceIDRequest :md5sum-) "983e371f5f8e03bba53608e89ded6b57")
(setf (get kortex_driver::SetDeviceIDRequest :datatype-) "kortex_driver/SetDeviceIDRequest")
(setf (get kortex_driver::SetDeviceIDRequest :definition-)
      "uint32 device_id
---

")

(setf (get kortex_driver::SetDeviceIDResponse :md5sum-) "983e371f5f8e03bba53608e89ded6b57")
(setf (get kortex_driver::SetDeviceIDResponse :datatype-) "kortex_driver/SetDeviceIDResponse")
(setf (get kortex_driver::SetDeviceIDResponse :definition-)
      "uint32 device_id
---

")



(provide :kortex_driver/SetDeviceID "983e371f5f8e03bba53608e89ded6b57")


