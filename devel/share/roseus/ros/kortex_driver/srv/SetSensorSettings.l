;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetSensorSettings)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetSensorSettings (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETSENSORSETTINGS")
  (make-package "KORTEX_DRIVER::SETSENSORSETTINGS"))
(unless (find-package "KORTEX_DRIVER::SETSENSORSETTINGSREQUEST")
  (make-package "KORTEX_DRIVER::SETSENSORSETTINGSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETSENSORSETTINGSRESPONSE")
  (make-package "KORTEX_DRIVER::SETSENSORSETTINGSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetSensorSettingsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetSensorSettingsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SensorSettings :init))
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
    ;; kortex_driver/SensorSettings _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SensorSettings _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SensorSettings _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetSensorSettingsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetSensorSettingsResponse
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

(defclass kortex_driver::SetSensorSettings
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetSensorSettings :md5sum-) "30daf04e01a3f6e06e8cd7b777f1164c")
(setf (get kortex_driver::SetSensorSettings :datatype-) "kortex_driver/SetSensorSettings")
(setf (get kortex_driver::SetSensorSettings :request) kortex_driver::SetSensorSettingsRequest)
(setf (get kortex_driver::SetSensorSettings :response) kortex_driver::SetSensorSettingsResponse)

(defmethod kortex_driver::SetSensorSettingsRequest
  (:response () (instance kortex_driver::SetSensorSettingsResponse :init)))

(setf (get kortex_driver::SetSensorSettingsRequest :md5sum-) "30daf04e01a3f6e06e8cd7b777f1164c")
(setf (get kortex_driver::SetSensorSettingsRequest :datatype-) "kortex_driver/SetSensorSettingsRequest")
(setf (get kortex_driver::SetSensorSettingsRequest :definition-)
      "SensorSettings input

================================================================================
MSG: kortex_driver/SensorSettings

uint32 sensor
uint32 resolution
uint32 frame_rate
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetSensorSettingsResponse :md5sum-) "30daf04e01a3f6e06e8cd7b777f1164c")
(setf (get kortex_driver::SetSensorSettingsResponse :datatype-) "kortex_driver/SetSensorSettingsResponse")
(setf (get kortex_driver::SetSensorSettingsResponse :definition-)
      "SensorSettings input

================================================================================
MSG: kortex_driver/SensorSettings

uint32 sensor
uint32 resolution
uint32 frame_rate
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetSensorSettings "30daf04e01a3f6e06e8cd7b777f1164c")


