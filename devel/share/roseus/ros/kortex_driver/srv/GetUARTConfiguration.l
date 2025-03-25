;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetUARTConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetUARTConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETUARTCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETUARTCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETUARTCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETUARTCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETUARTCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETUARTCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetUARTConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetUARTConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::UARTDeviceIdentification :init))
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
    ;; kortex_driver/UARTDeviceIdentification _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UARTDeviceIdentification _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UARTDeviceIdentification _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetUARTConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetUARTConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::UARTConfiguration :init))
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
    ;; kortex_driver/UARTConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UARTConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UARTConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetUARTConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetUARTConfiguration :md5sum-) "d041631488869c35b5eb142c6744c8a7")
(setf (get kortex_driver::GetUARTConfiguration :datatype-) "kortex_driver/GetUARTConfiguration")
(setf (get kortex_driver::GetUARTConfiguration :request) kortex_driver::GetUARTConfigurationRequest)
(setf (get kortex_driver::GetUARTConfiguration :response) kortex_driver::GetUARTConfigurationResponse)

(defmethod kortex_driver::GetUARTConfigurationRequest
  (:response () (instance kortex_driver::GetUARTConfigurationResponse :init)))

(setf (get kortex_driver::GetUARTConfigurationRequest :md5sum-) "d041631488869c35b5eb142c6744c8a7")
(setf (get kortex_driver::GetUARTConfigurationRequest :datatype-) "kortex_driver/GetUARTConfigurationRequest")
(setf (get kortex_driver::GetUARTConfigurationRequest :definition-)
      "UARTDeviceIdentification input

================================================================================
MSG: kortex_driver/UARTDeviceIdentification

---
UARTConfiguration output

================================================================================
MSG: kortex_driver/UARTConfiguration

uint32 port_id
bool enabled
uint32 speed
uint32 word_length
uint32 stop_bits
")

(setf (get kortex_driver::GetUARTConfigurationResponse :md5sum-) "d041631488869c35b5eb142c6744c8a7")
(setf (get kortex_driver::GetUARTConfigurationResponse :datatype-) "kortex_driver/GetUARTConfigurationResponse")
(setf (get kortex_driver::GetUARTConfigurationResponse :definition-)
      "UARTDeviceIdentification input

================================================================================
MSG: kortex_driver/UARTDeviceIdentification

---
UARTConfiguration output

================================================================================
MSG: kortex_driver/UARTConfiguration

uint32 port_id
bool enabled
uint32 speed
uint32 word_length
uint32 stop_bits
")



(provide :kortex_driver/GetUARTConfiguration "d041631488869c35b5eb142c6744c8a7")


