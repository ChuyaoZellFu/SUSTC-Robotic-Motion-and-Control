;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetGPIOConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetGPIOConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETGPIOCONFIGURATION")
  (make-package "KORTEX_DRIVER::SETGPIOCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETGPIOCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETGPIOCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETGPIOCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETGPIOCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetGPIOConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetGPIOConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::InterconnectConfig_GPIOConfiguration :init))
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
    ;; kortex_driver/InterconnectConfig_GPIOConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/InterconnectConfig_GPIOConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/InterconnectConfig_GPIOConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetGPIOConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetGPIOConfigurationResponse
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

(defclass kortex_driver::SetGPIOConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetGPIOConfiguration :md5sum-) "c67d61eec82191ca0955210766f4f427")
(setf (get kortex_driver::SetGPIOConfiguration :datatype-) "kortex_driver/SetGPIOConfiguration")
(setf (get kortex_driver::SetGPIOConfiguration :request) kortex_driver::SetGPIOConfigurationRequest)
(setf (get kortex_driver::SetGPIOConfiguration :response) kortex_driver::SetGPIOConfigurationResponse)

(defmethod kortex_driver::SetGPIOConfigurationRequest
  (:response () (instance kortex_driver::SetGPIOConfigurationResponse :init)))

(setf (get kortex_driver::SetGPIOConfigurationRequest :md5sum-) "c67d61eec82191ca0955210766f4f427")
(setf (get kortex_driver::SetGPIOConfigurationRequest :datatype-) "kortex_driver/SetGPIOConfigurationRequest")
(setf (get kortex_driver::SetGPIOConfigurationRequest :definition-)
      "InterconnectConfig_GPIOConfiguration input

================================================================================
MSG: kortex_driver/InterconnectConfig_GPIOConfiguration

uint32 identifier
uint32 mode
uint32 pull
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetGPIOConfigurationResponse :md5sum-) "c67d61eec82191ca0955210766f4f427")
(setf (get kortex_driver::SetGPIOConfigurationResponse :datatype-) "kortex_driver/SetGPIOConfigurationResponse")
(setf (get kortex_driver::SetGPIOConfigurationResponse :definition-)
      "InterconnectConfig_GPIOConfiguration input

================================================================================
MSG: kortex_driver/InterconnectConfig_GPIOConfiguration

uint32 identifier
uint32 mode
uint32 pull
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetGPIOConfiguration "c67d61eec82191ca0955210766f4f427")


