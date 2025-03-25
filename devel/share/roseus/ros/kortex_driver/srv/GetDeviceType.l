;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetDeviceType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetDeviceType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETDEVICETYPE")
  (make-package "KORTEX_DRIVER::GETDEVICETYPE"))
(unless (find-package "KORTEX_DRIVER::GETDEVICETYPEREQUEST")
  (make-package "KORTEX_DRIVER::GETDEVICETYPEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETDEVICETYPERESPONSE")
  (make-package "KORTEX_DRIVER::GETDEVICETYPERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetDeviceTypeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetDeviceTypeRequest
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

(defclass kortex_driver::GetDeviceTypeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetDeviceTypeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::DeviceType :init))
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
    ;; kortex_driver/DeviceType _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/DeviceType _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/DeviceType _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetDeviceType
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetDeviceType :md5sum-) "2d4eec40c5cb478115bd33d8df8d00b7")
(setf (get kortex_driver::GetDeviceType :datatype-) "kortex_driver/GetDeviceType")
(setf (get kortex_driver::GetDeviceType :request) kortex_driver::GetDeviceTypeRequest)
(setf (get kortex_driver::GetDeviceType :response) kortex_driver::GetDeviceTypeResponse)

(defmethod kortex_driver::GetDeviceTypeRequest
  (:response () (instance kortex_driver::GetDeviceTypeResponse :init)))

(setf (get kortex_driver::GetDeviceTypeRequest :md5sum-) "2d4eec40c5cb478115bd33d8df8d00b7")
(setf (get kortex_driver::GetDeviceTypeRequest :datatype-) "kortex_driver/GetDeviceTypeRequest")
(setf (get kortex_driver::GetDeviceTypeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
DeviceType output

================================================================================
MSG: kortex_driver/DeviceType

")

(setf (get kortex_driver::GetDeviceTypeResponse :md5sum-) "2d4eec40c5cb478115bd33d8df8d00b7")
(setf (get kortex_driver::GetDeviceTypeResponse :datatype-) "kortex_driver/GetDeviceTypeResponse")
(setf (get kortex_driver::GetDeviceTypeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
DeviceType output

================================================================================
MSG: kortex_driver/DeviceType

")



(provide :kortex_driver/GetDeviceType "2d4eec40c5cb478115bd33d8df8d00b7")


