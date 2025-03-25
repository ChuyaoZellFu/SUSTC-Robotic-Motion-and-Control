;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetControllerConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetControllerConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetControllerConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetControllerConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControllerHandle :init))
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
    ;; kortex_driver/ControllerHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetControllerConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetControllerConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ControllerConfiguration :init))
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
    ;; kortex_driver/ControllerConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetControllerConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetControllerConfiguration :md5sum-) "486c0887673fbfba27c914b77b97b479")
(setf (get kortex_driver::GetControllerConfiguration :datatype-) "kortex_driver/GetControllerConfiguration")
(setf (get kortex_driver::GetControllerConfiguration :request) kortex_driver::GetControllerConfigurationRequest)
(setf (get kortex_driver::GetControllerConfiguration :response) kortex_driver::GetControllerConfigurationResponse)

(defmethod kortex_driver::GetControllerConfigurationRequest
  (:response () (instance kortex_driver::GetControllerConfigurationResponse :init)))

(setf (get kortex_driver::GetControllerConfigurationRequest :md5sum-) "486c0887673fbfba27c914b77b97b479")
(setf (get kortex_driver::GetControllerConfigurationRequest :datatype-) "kortex_driver/GetControllerConfigurationRequest")
(setf (get kortex_driver::GetControllerConfigurationRequest :definition-)
      "ControllerHandle input

================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
---
ControllerConfiguration output

================================================================================
MSG: kortex_driver/ControllerConfiguration

ControllerHandle handle
string name
MappingHandle active_mapping_handle
string analog_input_identifier_enum
string digital_input_identifier_enum
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
")

(setf (get kortex_driver::GetControllerConfigurationResponse :md5sum-) "486c0887673fbfba27c914b77b97b479")
(setf (get kortex_driver::GetControllerConfigurationResponse :datatype-) "kortex_driver/GetControllerConfigurationResponse")
(setf (get kortex_driver::GetControllerConfigurationResponse :definition-)
      "ControllerHandle input

================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
---
ControllerConfiguration output

================================================================================
MSG: kortex_driver/ControllerConfiguration

ControllerHandle handle
string name
MappingHandle active_mapping_handle
string analog_input_identifier_enum
string digital_input_identifier_enum
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
")



(provide :kortex_driver/GetControllerConfiguration "486c0887673fbfba27c914b77b97b479")


