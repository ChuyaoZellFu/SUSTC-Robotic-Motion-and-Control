;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetGPIOConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetGPIOConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETGPIOCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETGPIOCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETGPIOCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETGPIOCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETGPIOCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETGPIOCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetGPIOConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetGPIOConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::GPIOIdentification :init))
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
    ;; kortex_driver/GPIOIdentification _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GPIOIdentification _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GPIOIdentification _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetGPIOConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetGPIOConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::InterconnectConfig_GPIOConfiguration :init))
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
    ;; kortex_driver/InterconnectConfig_GPIOConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/InterconnectConfig_GPIOConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/InterconnectConfig_GPIOConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetGPIOConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetGPIOConfiguration :md5sum-) "54322e7ee9a9cb34b1b8f34c41c5ae4d")
(setf (get kortex_driver::GetGPIOConfiguration :datatype-) "kortex_driver/GetGPIOConfiguration")
(setf (get kortex_driver::GetGPIOConfiguration :request) kortex_driver::GetGPIOConfigurationRequest)
(setf (get kortex_driver::GetGPIOConfiguration :response) kortex_driver::GetGPIOConfigurationResponse)

(defmethod kortex_driver::GetGPIOConfigurationRequest
  (:response () (instance kortex_driver::GetGPIOConfigurationResponse :init)))

(setf (get kortex_driver::GetGPIOConfigurationRequest :md5sum-) "54322e7ee9a9cb34b1b8f34c41c5ae4d")
(setf (get kortex_driver::GetGPIOConfigurationRequest :datatype-) "kortex_driver/GetGPIOConfigurationRequest")
(setf (get kortex_driver::GetGPIOConfigurationRequest :definition-)
      "GPIOIdentification input

================================================================================
MSG: kortex_driver/GPIOIdentification

---
InterconnectConfig_GPIOConfiguration output

================================================================================
MSG: kortex_driver/InterconnectConfig_GPIOConfiguration

uint32 identifier
uint32 mode
uint32 pull
")

(setf (get kortex_driver::GetGPIOConfigurationResponse :md5sum-) "54322e7ee9a9cb34b1b8f34c41c5ae4d")
(setf (get kortex_driver::GetGPIOConfigurationResponse :datatype-) "kortex_driver/GetGPIOConfigurationResponse")
(setf (get kortex_driver::GetGPIOConfigurationResponse :definition-)
      "GPIOIdentification input

================================================================================
MSG: kortex_driver/GPIOIdentification

---
InterconnectConfig_GPIOConfiguration output

================================================================================
MSG: kortex_driver/InterconnectConfig_GPIOConfiguration

uint32 identifier
uint32 mode
uint32 pull
")



(provide :kortex_driver/GetGPIOConfiguration "54322e7ee9a9cb34b1b8f34c41c5ae4d")


