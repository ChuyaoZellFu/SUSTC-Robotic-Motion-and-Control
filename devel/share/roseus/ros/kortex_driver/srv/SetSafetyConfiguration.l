;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetSafetyConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetSafetyConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETSAFETYCONFIGURATION")
  (make-package "KORTEX_DRIVER::SETSAFETYCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETSAFETYCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETSAFETYCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetSafetyConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetSafetyConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SafetyConfiguration :init))
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
    ;; kortex_driver/SafetyConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetSafetyConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetSafetyConfigurationResponse
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

(defclass kortex_driver::SetSafetyConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetSafetyConfiguration :md5sum-) "b1a9f41cb42ab39cba6bfb32d20fea4f")
(setf (get kortex_driver::SetSafetyConfiguration :datatype-) "kortex_driver/SetSafetyConfiguration")
(setf (get kortex_driver::SetSafetyConfiguration :request) kortex_driver::SetSafetyConfigurationRequest)
(setf (get kortex_driver::SetSafetyConfiguration :response) kortex_driver::SetSafetyConfigurationResponse)

(defmethod kortex_driver::SetSafetyConfigurationRequest
  (:response () (instance kortex_driver::SetSafetyConfigurationResponse :init)))

(setf (get kortex_driver::SetSafetyConfigurationRequest :md5sum-) "b1a9f41cb42ab39cba6bfb32d20fea4f")
(setf (get kortex_driver::SetSafetyConfigurationRequest :datatype-) "kortex_driver/SetSafetyConfigurationRequest")
(setf (get kortex_driver::SetSafetyConfigurationRequest :definition-)
      "SafetyConfiguration input

================================================================================
MSG: kortex_driver/SafetyConfiguration

SafetyHandle handle
float32 error_threshold
float32 warning_threshold
SafetyEnable enable
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetSafetyConfigurationResponse :md5sum-) "b1a9f41cb42ab39cba6bfb32d20fea4f")
(setf (get kortex_driver::SetSafetyConfigurationResponse :datatype-) "kortex_driver/SetSafetyConfigurationResponse")
(setf (get kortex_driver::SetSafetyConfigurationResponse :definition-)
      "SafetyConfiguration input

================================================================================
MSG: kortex_driver/SafetyConfiguration

SafetyHandle handle
float32 error_threshold
float32 warning_threshold
SafetyEnable enable
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetSafetyConfiguration "b1a9f41cb42ab39cba6bfb32d20fea4f")


