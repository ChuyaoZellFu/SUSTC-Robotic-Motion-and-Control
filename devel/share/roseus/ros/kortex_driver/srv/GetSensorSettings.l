;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetSensorSettings)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetSensorSettings (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSENSORSETTINGS")
  (make-package "KORTEX_DRIVER::GETSENSORSETTINGS"))
(unless (find-package "KORTEX_DRIVER::GETSENSORSETTINGSREQUEST")
  (make-package "KORTEX_DRIVER::GETSENSORSETTINGSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSENSORSETTINGSRESPONSE")
  (make-package "KORTEX_DRIVER::GETSENSORSETTINGSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetSensorSettingsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetSensorSettingsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SensorIdentifier :init))
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
    ;; kortex_driver/SensorIdentifier _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SensorIdentifier _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SensorIdentifier _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSensorSettingsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetSensorSettingsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SensorSettings :init))
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
    ;; kortex_driver/SensorSettings _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SensorSettings _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SensorSettings _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSensorSettings
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetSensorSettings :md5sum-) "16278c9fd5ce95a2e5bda4211043968e")
(setf (get kortex_driver::GetSensorSettings :datatype-) "kortex_driver/GetSensorSettings")
(setf (get kortex_driver::GetSensorSettings :request) kortex_driver::GetSensorSettingsRequest)
(setf (get kortex_driver::GetSensorSettings :response) kortex_driver::GetSensorSettingsResponse)

(defmethod kortex_driver::GetSensorSettingsRequest
  (:response () (instance kortex_driver::GetSensorSettingsResponse :init)))

(setf (get kortex_driver::GetSensorSettingsRequest :md5sum-) "16278c9fd5ce95a2e5bda4211043968e")
(setf (get kortex_driver::GetSensorSettingsRequest :datatype-) "kortex_driver/GetSensorSettingsRequest")
(setf (get kortex_driver::GetSensorSettingsRequest :definition-)
      "SensorIdentifier input

================================================================================
MSG: kortex_driver/SensorIdentifier

---
SensorSettings output

================================================================================
MSG: kortex_driver/SensorSettings

uint32 sensor
uint32 resolution
uint32 frame_rate
")

(setf (get kortex_driver::GetSensorSettingsResponse :md5sum-) "16278c9fd5ce95a2e5bda4211043968e")
(setf (get kortex_driver::GetSensorSettingsResponse :datatype-) "kortex_driver/GetSensorSettingsResponse")
(setf (get kortex_driver::GetSensorSettingsResponse :definition-)
      "SensorIdentifier input

================================================================================
MSG: kortex_driver/SensorIdentifier

---
SensorSettings output

================================================================================
MSG: kortex_driver/SensorSettings

uint32 sensor
uint32 resolution
uint32 frame_rate
")



(provide :kortex_driver/GetSensorSettings "16278c9fd5ce95a2e5bda4211043968e")


