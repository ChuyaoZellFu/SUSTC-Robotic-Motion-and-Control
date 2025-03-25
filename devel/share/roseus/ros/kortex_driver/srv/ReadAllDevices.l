;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadAllDevices)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadAllDevices (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READALLDEVICES")
  (make-package "KORTEX_DRIVER::READALLDEVICES"))
(unless (find-package "KORTEX_DRIVER::READALLDEVICESREQUEST")
  (make-package "KORTEX_DRIVER::READALLDEVICESREQUEST"))
(unless (find-package "KORTEX_DRIVER::READALLDEVICESRESPONSE")
  (make-package "KORTEX_DRIVER::READALLDEVICESRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadAllDevicesRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadAllDevicesRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadAllDevicesResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadAllDevicesResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::DeviceHandles :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/DeviceHandles _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/DeviceHandles _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/DeviceHandles _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadAllDevices
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadAllDevices :md5sum-) "9f8a3530be8da22ec93eee7ab47ef9a0")
(setf (get kortex_driver::ReadAllDevices :datatype-) "kortex_driver/ReadAllDevices")
(setf (get kortex_driver::ReadAllDevices :request) kortex_driver::ReadAllDevicesRequest)
(setf (get kortex_driver::ReadAllDevices :response) kortex_driver::ReadAllDevicesResponse)

(defmethod kortex_driver::ReadAllDevicesRequest
  (:response () (instance kortex_driver::ReadAllDevicesResponse :init)))

(setf (get kortex_driver::ReadAllDevicesRequest :md5sum-) "9f8a3530be8da22ec93eee7ab47ef9a0")
(setf (get kortex_driver::ReadAllDevicesRequest :datatype-) "kortex_driver/ReadAllDevicesRequest")
(setf (get kortex_driver::ReadAllDevicesRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
DeviceHandles output

================================================================================
MSG: kortex_driver/DeviceHandles

DeviceHandle[] device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
")

(setf (get kortex_driver::ReadAllDevicesResponse :md5sum-) "9f8a3530be8da22ec93eee7ab47ef9a0")
(setf (get kortex_driver::ReadAllDevicesResponse :datatype-) "kortex_driver/ReadAllDevicesResponse")
(setf (get kortex_driver::ReadAllDevicesResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
DeviceHandles output

================================================================================
MSG: kortex_driver/DeviceHandles

DeviceHandle[] device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
")



(provide :kortex_driver/ReadAllDevices "9f8a3530be8da22ec93eee7ab47ef9a0")


