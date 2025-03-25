;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeviceHandles)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeviceHandles (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DEVICEHANDLES")
  (make-package "KORTEX_DRIVER::DEVICEHANDLES"))

(in-package "ROS")
;;//! \htmlinclude DeviceHandles.msg.html


(defclass kortex_driver::DeviceHandles
  :super ros::object
  :slots (_device_handle ))

(defmethod kortex_driver::DeviceHandles
  (:init
   (&key
    ((:device_handle __device_handle) ())
    )
   (send-super :init)
   (setq _device_handle __device_handle)
   self)
  (:device_handle
   (&rest __device_handle)
   (if (keywordp (car __device_handle))
       (send* _device_handle __device_handle)
     (progn
       (if __device_handle (setq _device_handle (car __device_handle)))
       _device_handle)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/DeviceHandle[] _device_handle
    (apply #'+ (send-all _device_handle :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/DeviceHandle[] _device_handle
     (write-long (length _device_handle) s)
     (dolist (elem _device_handle)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/DeviceHandle[] _device_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _device_handle (let (r) (dotimes (i n) (push (instance kortex_driver::DeviceHandle :init) r)) r))
     (dolist (elem- _device_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::DeviceHandles :md5sum-) "ad854635e0d722ce08228e35a86c0173")
(setf (get kortex_driver::DeviceHandles :datatype-) "kortex_driver/DeviceHandles")
(setf (get kortex_driver::DeviceHandles :definition-)
      "
DeviceHandle[] device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
uint32 order
")



(provide :kortex_driver/DeviceHandles "ad854635e0d722ce08228e35a86c0173")


