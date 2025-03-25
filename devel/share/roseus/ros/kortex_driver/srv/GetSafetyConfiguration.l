;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetSafetyConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetSafetyConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSAFETYCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETSAFETYCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETSAFETYCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETSAFETYCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetSafetyConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetSafetyConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SafetyHandle :init))
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
    ;; kortex_driver/SafetyHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSafetyConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetSafetyConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SafetyConfiguration :init))
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
    ;; kortex_driver/SafetyConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSafetyConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetSafetyConfiguration :md5sum-) "943b22eac92db7c1fc8c0b5bbdfb1565")
(setf (get kortex_driver::GetSafetyConfiguration :datatype-) "kortex_driver/GetSafetyConfiguration")
(setf (get kortex_driver::GetSafetyConfiguration :request) kortex_driver::GetSafetyConfigurationRequest)
(setf (get kortex_driver::GetSafetyConfiguration :response) kortex_driver::GetSafetyConfigurationResponse)

(defmethod kortex_driver::GetSafetyConfigurationRequest
  (:response () (instance kortex_driver::GetSafetyConfigurationResponse :init)))

(setf (get kortex_driver::GetSafetyConfigurationRequest :md5sum-) "943b22eac92db7c1fc8c0b5bbdfb1565")
(setf (get kortex_driver::GetSafetyConfigurationRequest :datatype-) "kortex_driver/GetSafetyConfigurationRequest")
(setf (get kortex_driver::GetSafetyConfigurationRequest :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyConfiguration output

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
")

(setf (get kortex_driver::GetSafetyConfigurationResponse :md5sum-) "943b22eac92db7c1fc8c0b5bbdfb1565")
(setf (get kortex_driver::GetSafetyConfigurationResponse :datatype-) "kortex_driver/GetSafetyConfigurationResponse")
(setf (get kortex_driver::GetSafetyConfigurationResponse :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyConfiguration output

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
")



(provide :kortex_driver/GetSafetyConfiguration "943b22eac92db7c1fc8c0b5bbdfb1565")


