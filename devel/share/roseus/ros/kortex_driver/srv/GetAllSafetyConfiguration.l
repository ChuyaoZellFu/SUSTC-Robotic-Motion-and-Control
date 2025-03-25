;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllSafetyConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllSafetyConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLSAFETYCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETALLSAFETYCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETALLSAFETYCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETALLSAFETYCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLSAFETYCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLSAFETYCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllSafetyConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllSafetyConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllSafetyConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllSafetyConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SafetyConfigurationList :init))
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
    ;; kortex_driver/SafetyConfigurationList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyConfigurationList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyConfigurationList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllSafetyConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllSafetyConfiguration :md5sum-) "332c0fb8c1c499e5ec5c674488e0110f")
(setf (get kortex_driver::GetAllSafetyConfiguration :datatype-) "kortex_driver/GetAllSafetyConfiguration")
(setf (get kortex_driver::GetAllSafetyConfiguration :request) kortex_driver::GetAllSafetyConfigurationRequest)
(setf (get kortex_driver::GetAllSafetyConfiguration :response) kortex_driver::GetAllSafetyConfigurationResponse)

(defmethod kortex_driver::GetAllSafetyConfigurationRequest
  (:response () (instance kortex_driver::GetAllSafetyConfigurationResponse :init)))

(setf (get kortex_driver::GetAllSafetyConfigurationRequest :md5sum-) "332c0fb8c1c499e5ec5c674488e0110f")
(setf (get kortex_driver::GetAllSafetyConfigurationRequest :datatype-) "kortex_driver/GetAllSafetyConfigurationRequest")
(setf (get kortex_driver::GetAllSafetyConfigurationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
SafetyConfigurationList output

================================================================================
MSG: kortex_driver/SafetyConfigurationList

SafetyConfiguration[] configuration
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

(setf (get kortex_driver::GetAllSafetyConfigurationResponse :md5sum-) "332c0fb8c1c499e5ec5c674488e0110f")
(setf (get kortex_driver::GetAllSafetyConfigurationResponse :datatype-) "kortex_driver/GetAllSafetyConfigurationResponse")
(setf (get kortex_driver::GetAllSafetyConfigurationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
SafetyConfigurationList output

================================================================================
MSG: kortex_driver/SafetyConfigurationList

SafetyConfiguration[] configuration
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



(provide :kortex_driver/GetAllSafetyConfiguration "332c0fb8c1c499e5ec5c674488e0110f")


