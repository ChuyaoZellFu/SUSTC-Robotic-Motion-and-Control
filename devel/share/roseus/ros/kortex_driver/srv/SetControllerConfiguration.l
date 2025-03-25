;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetControllerConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetControllerConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATION")
  (make-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetControllerConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetControllerConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControllerConfiguration :init))
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
    ;; kortex_driver/ControllerConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetControllerConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetControllerConfigurationResponse
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

(defclass kortex_driver::SetControllerConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetControllerConfiguration :md5sum-) "4158c0b1ce15d5824908b5157fccd994")
(setf (get kortex_driver::SetControllerConfiguration :datatype-) "kortex_driver/SetControllerConfiguration")
(setf (get kortex_driver::SetControllerConfiguration :request) kortex_driver::SetControllerConfigurationRequest)
(setf (get kortex_driver::SetControllerConfiguration :response) kortex_driver::SetControllerConfigurationResponse)

(defmethod kortex_driver::SetControllerConfigurationRequest
  (:response () (instance kortex_driver::SetControllerConfigurationResponse :init)))

(setf (get kortex_driver::SetControllerConfigurationRequest :md5sum-) "4158c0b1ce15d5824908b5157fccd994")
(setf (get kortex_driver::SetControllerConfigurationRequest :datatype-) "kortex_driver/SetControllerConfigurationRequest")
(setf (get kortex_driver::SetControllerConfigurationRequest :definition-)
      "ControllerConfiguration input

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
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetControllerConfigurationResponse :md5sum-) "4158c0b1ce15d5824908b5157fccd994")
(setf (get kortex_driver::SetControllerConfigurationResponse :datatype-) "kortex_driver/SetControllerConfigurationResponse")
(setf (get kortex_driver::SetControllerConfigurationResponse :definition-)
      "ControllerConfiguration input

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
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetControllerConfiguration "4158c0b1ce15d5824908b5157fccd994")


