;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetOperatingMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetOperatingMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETOPERATINGMODE")
  (make-package "KORTEX_DRIVER::GETOPERATINGMODE"))
(unless (find-package "KORTEX_DRIVER::GETOPERATINGMODEREQUEST")
  (make-package "KORTEX_DRIVER::GETOPERATINGMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETOPERATINGMODERESPONSE")
  (make-package "KORTEX_DRIVER::GETOPERATINGMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetOperatingModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetOperatingModeRequest
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

(defclass kortex_driver::GetOperatingModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetOperatingModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::OperatingModeInformation :init))
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
    ;; kortex_driver/OperatingModeInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/OperatingModeInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/OperatingModeInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetOperatingMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetOperatingMode :md5sum-) "fe14b4b15e0f48ffd6cf705ff5a59c40")
(setf (get kortex_driver::GetOperatingMode :datatype-) "kortex_driver/GetOperatingMode")
(setf (get kortex_driver::GetOperatingMode :request) kortex_driver::GetOperatingModeRequest)
(setf (get kortex_driver::GetOperatingMode :response) kortex_driver::GetOperatingModeResponse)

(defmethod kortex_driver::GetOperatingModeRequest
  (:response () (instance kortex_driver::GetOperatingModeResponse :init)))

(setf (get kortex_driver::GetOperatingModeRequest :md5sum-) "fe14b4b15e0f48ffd6cf705ff5a59c40")
(setf (get kortex_driver::GetOperatingModeRequest :datatype-) "kortex_driver/GetOperatingModeRequest")
(setf (get kortex_driver::GetOperatingModeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
OperatingModeInformation output

================================================================================
MSG: kortex_driver/OperatingModeInformation

uint32 operating_mode
DeviceHandle device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
")

(setf (get kortex_driver::GetOperatingModeResponse :md5sum-) "fe14b4b15e0f48ffd6cf705ff5a59c40")
(setf (get kortex_driver::GetOperatingModeResponse :datatype-) "kortex_driver/GetOperatingModeResponse")
(setf (get kortex_driver::GetOperatingModeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
OperatingModeInformation output

================================================================================
MSG: kortex_driver/OperatingModeInformation

uint32 operating_mode
DeviceHandle device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
")



(provide :kortex_driver/GetOperatingMode "fe14b4b15e0f48ffd6cf705ff5a59c40")


