;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetI2CConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetI2CConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETI2CCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETI2CCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETI2CCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETI2CCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETI2CCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETI2CCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetI2CConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetI2CConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::I2CDeviceIdentification :init))
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
    ;; kortex_driver/I2CDeviceIdentification _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CDeviceIdentification _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CDeviceIdentification _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetI2CConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetI2CConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::I2CConfiguration :init))
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
    ;; kortex_driver/I2CConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetI2CConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetI2CConfiguration :md5sum-) "d50c46d9bec318dcd7281869a6e5606a")
(setf (get kortex_driver::GetI2CConfiguration :datatype-) "kortex_driver/GetI2CConfiguration")
(setf (get kortex_driver::GetI2CConfiguration :request) kortex_driver::GetI2CConfigurationRequest)
(setf (get kortex_driver::GetI2CConfiguration :response) kortex_driver::GetI2CConfigurationResponse)

(defmethod kortex_driver::GetI2CConfigurationRequest
  (:response () (instance kortex_driver::GetI2CConfigurationResponse :init)))

(setf (get kortex_driver::GetI2CConfigurationRequest :md5sum-) "d50c46d9bec318dcd7281869a6e5606a")
(setf (get kortex_driver::GetI2CConfigurationRequest :datatype-) "kortex_driver/GetI2CConfigurationRequest")
(setf (get kortex_driver::GetI2CConfigurationRequest :definition-)
      "I2CDeviceIdentification input

================================================================================
MSG: kortex_driver/I2CDeviceIdentification

---
I2CConfiguration output

================================================================================
MSG: kortex_driver/I2CConfiguration

uint32 device
bool enabled
uint32 mode
")

(setf (get kortex_driver::GetI2CConfigurationResponse :md5sum-) "d50c46d9bec318dcd7281869a6e5606a")
(setf (get kortex_driver::GetI2CConfigurationResponse :datatype-) "kortex_driver/GetI2CConfigurationResponse")
(setf (get kortex_driver::GetI2CConfigurationResponse :definition-)
      "I2CDeviceIdentification input

================================================================================
MSG: kortex_driver/I2CDeviceIdentification

---
I2CConfiguration output

================================================================================
MSG: kortex_driver/I2CConfiguration

uint32 device
bool enabled
uint32 mode
")



(provide :kortex_driver/GetI2CConfiguration "d50c46d9bec318dcd7281869a6e5606a")


