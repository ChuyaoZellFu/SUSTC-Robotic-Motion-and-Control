;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetI2CConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetI2CConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETI2CCONFIGURATION")
  (make-package "KORTEX_DRIVER::SETI2CCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETI2CCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETI2CCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETI2CCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETI2CCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetI2CConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetI2CConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::I2CConfiguration :init))
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
    ;; kortex_driver/I2CConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetI2CConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetI2CConfigurationResponse
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

(defclass kortex_driver::SetI2CConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetI2CConfiguration :md5sum-) "fcaaa1a0d5ba74e939b1b6f1a0988c2a")
(setf (get kortex_driver::SetI2CConfiguration :datatype-) "kortex_driver/SetI2CConfiguration")
(setf (get kortex_driver::SetI2CConfiguration :request) kortex_driver::SetI2CConfigurationRequest)
(setf (get kortex_driver::SetI2CConfiguration :response) kortex_driver::SetI2CConfigurationResponse)

(defmethod kortex_driver::SetI2CConfigurationRequest
  (:response () (instance kortex_driver::SetI2CConfigurationResponse :init)))

(setf (get kortex_driver::SetI2CConfigurationRequest :md5sum-) "fcaaa1a0d5ba74e939b1b6f1a0988c2a")
(setf (get kortex_driver::SetI2CConfigurationRequest :datatype-) "kortex_driver/SetI2CConfigurationRequest")
(setf (get kortex_driver::SetI2CConfigurationRequest :definition-)
      "I2CConfiguration input

================================================================================
MSG: kortex_driver/I2CConfiguration

uint32 device
bool enabled
uint32 mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetI2CConfigurationResponse :md5sum-) "fcaaa1a0d5ba74e939b1b6f1a0988c2a")
(setf (get kortex_driver::SetI2CConfigurationResponse :datatype-) "kortex_driver/SetI2CConfigurationResponse")
(setf (get kortex_driver::SetI2CConfigurationResponse :definition-)
      "I2CConfiguration input

================================================================================
MSG: kortex_driver/I2CConfiguration

uint32 device
bool enabled
uint32 mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetI2CConfiguration "fcaaa1a0d5ba74e939b1b6f1a0988c2a")


