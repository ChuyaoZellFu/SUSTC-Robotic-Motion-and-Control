;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetUARTConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetUARTConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETUARTCONFIGURATION")
  (make-package "KORTEX_DRIVER::SETUARTCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETUARTCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETUARTCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETUARTCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETUARTCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetUARTConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetUARTConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::UARTConfiguration :init))
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
    ;; kortex_driver/UARTConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UARTConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UARTConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetUARTConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetUARTConfigurationResponse
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

(defclass kortex_driver::SetUARTConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetUARTConfiguration :md5sum-) "44945f03b0c880ed6bb57eedba493047")
(setf (get kortex_driver::SetUARTConfiguration :datatype-) "kortex_driver/SetUARTConfiguration")
(setf (get kortex_driver::SetUARTConfiguration :request) kortex_driver::SetUARTConfigurationRequest)
(setf (get kortex_driver::SetUARTConfiguration :response) kortex_driver::SetUARTConfigurationResponse)

(defmethod kortex_driver::SetUARTConfigurationRequest
  (:response () (instance kortex_driver::SetUARTConfigurationResponse :init)))

(setf (get kortex_driver::SetUARTConfigurationRequest :md5sum-) "44945f03b0c880ed6bb57eedba493047")
(setf (get kortex_driver::SetUARTConfigurationRequest :datatype-) "kortex_driver/SetUARTConfigurationRequest")
(setf (get kortex_driver::SetUARTConfigurationRequest :definition-)
      "UARTConfiguration input

================================================================================
MSG: kortex_driver/UARTConfiguration

uint32 port_id
bool enabled
uint32 speed
uint32 word_length
uint32 stop_bits
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetUARTConfigurationResponse :md5sum-) "44945f03b0c880ed6bb57eedba493047")
(setf (get kortex_driver::SetUARTConfigurationResponse :datatype-) "kortex_driver/SetUARTConfigurationResponse")
(setf (get kortex_driver::SetUARTConfigurationResponse :definition-)
      "UARTConfiguration input

================================================================================
MSG: kortex_driver/UARTConfiguration

uint32 port_id
bool enabled
uint32 speed
uint32 word_length
uint32 stop_bits
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetUARTConfiguration "44945f03b0c880ed6bb57eedba493047")


