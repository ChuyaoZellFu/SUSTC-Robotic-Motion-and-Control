;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetOperatingMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetOperatingMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETOPERATINGMODE")
  (make-package "KORTEX_DRIVER::SETOPERATINGMODE"))
(unless (find-package "KORTEX_DRIVER::SETOPERATINGMODEREQUEST")
  (make-package "KORTEX_DRIVER::SETOPERATINGMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETOPERATINGMODERESPONSE")
  (make-package "KORTEX_DRIVER::SETOPERATINGMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetOperatingModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetOperatingModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::OperatingModeInformation :init))
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
    ;; kortex_driver/OperatingModeInformation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/OperatingModeInformation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/OperatingModeInformation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetOperatingModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetOperatingModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetOperatingMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetOperatingMode :md5sum-) "c4131bf69b8a60653e5e053fa2ce9151")
(setf (get kortex_driver::SetOperatingMode :datatype-) "kortex_driver/SetOperatingMode")
(setf (get kortex_driver::SetOperatingMode :request) kortex_driver::SetOperatingModeRequest)
(setf (get kortex_driver::SetOperatingMode :response) kortex_driver::SetOperatingModeResponse)

(defmethod kortex_driver::SetOperatingModeRequest
  (:response () (instance kortex_driver::SetOperatingModeResponse :init)))

(setf (get kortex_driver::SetOperatingModeRequest :md5sum-) "c4131bf69b8a60653e5e053fa2ce9151")
(setf (get kortex_driver::SetOperatingModeRequest :datatype-) "kortex_driver/SetOperatingModeRequest")
(setf (get kortex_driver::SetOperatingModeRequest :definition-)
      "OperatingModeInformation input

================================================================================
MSG: kortex_driver/OperatingModeInformation

uint32 operating_mode
DeviceHandle device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetOperatingModeResponse :md5sum-) "c4131bf69b8a60653e5e053fa2ce9151")
(setf (get kortex_driver::SetOperatingModeResponse :datatype-) "kortex_driver/SetOperatingModeResponse")
(setf (get kortex_driver::SetOperatingModeResponse :definition-)
      "OperatingModeInformation input

================================================================================
MSG: kortex_driver/OperatingModeInformation

uint32 operating_mode
DeviceHandle device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetOperatingMode "c4131bf69b8a60653e5e053fa2ce9151")


