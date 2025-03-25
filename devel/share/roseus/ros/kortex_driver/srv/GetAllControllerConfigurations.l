;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllControllerConfigurations)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllControllerConfigurations (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLCONTROLLERCONFIGURATIONS")
  (make-package "KORTEX_DRIVER::GETALLCONTROLLERCONFIGURATIONS"))
(unless (find-package "KORTEX_DRIVER::GETALLCONTROLLERCONFIGURATIONSREQUEST")
  (make-package "KORTEX_DRIVER::GETALLCONTROLLERCONFIGURATIONSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLCONTROLLERCONFIGURATIONSRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLCONTROLLERCONFIGURATIONSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllControllerConfigurationsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllControllerConfigurationsRequest
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

(defclass kortex_driver::GetAllControllerConfigurationsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllControllerConfigurationsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ControllerConfigurationList :init))
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
    ;; kortex_driver/ControllerConfigurationList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerConfigurationList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerConfigurationList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllControllerConfigurations
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllControllerConfigurations :md5sum-) "48bc932dac09542b910dbdc1482df5ca")
(setf (get kortex_driver::GetAllControllerConfigurations :datatype-) "kortex_driver/GetAllControllerConfigurations")
(setf (get kortex_driver::GetAllControllerConfigurations :request) kortex_driver::GetAllControllerConfigurationsRequest)
(setf (get kortex_driver::GetAllControllerConfigurations :response) kortex_driver::GetAllControllerConfigurationsResponse)

(defmethod kortex_driver::GetAllControllerConfigurationsRequest
  (:response () (instance kortex_driver::GetAllControllerConfigurationsResponse :init)))

(setf (get kortex_driver::GetAllControllerConfigurationsRequest :md5sum-) "48bc932dac09542b910dbdc1482df5ca")
(setf (get kortex_driver::GetAllControllerConfigurationsRequest :datatype-) "kortex_driver/GetAllControllerConfigurationsRequest")
(setf (get kortex_driver::GetAllControllerConfigurationsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControllerConfigurationList output

================================================================================
MSG: kortex_driver/ControllerConfigurationList

ControllerConfiguration[] controller_configurations
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

(setf (get kortex_driver::GetAllControllerConfigurationsResponse :md5sum-) "48bc932dac09542b910dbdc1482df5ca")
(setf (get kortex_driver::GetAllControllerConfigurationsResponse :datatype-) "kortex_driver/GetAllControllerConfigurationsResponse")
(setf (get kortex_driver::GetAllControllerConfigurationsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControllerConfigurationList output

================================================================================
MSG: kortex_driver/ControllerConfigurationList

ControllerConfiguration[] controller_configurations
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



(provide :kortex_driver/GetAllControllerConfigurations "48bc932dac09542b910dbdc1482df5ca")


