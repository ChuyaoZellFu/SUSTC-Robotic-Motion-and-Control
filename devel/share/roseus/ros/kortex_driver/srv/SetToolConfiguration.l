;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetToolConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetToolConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETTOOLCONFIGURATION")
  (make-package "KORTEX_DRIVER::SETTOOLCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETTOOLCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETTOOLCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETTOOLCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETTOOLCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetToolConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetToolConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ToolConfiguration :init))
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
    ;; kortex_driver/ToolConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ToolConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ToolConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetToolConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetToolConfigurationResponse
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

(defclass kortex_driver::SetToolConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetToolConfiguration :md5sum-) "d0f38c8ed77e0310c120105db58e4900")
(setf (get kortex_driver::SetToolConfiguration :datatype-) "kortex_driver/SetToolConfiguration")
(setf (get kortex_driver::SetToolConfiguration :request) kortex_driver::SetToolConfigurationRequest)
(setf (get kortex_driver::SetToolConfiguration :response) kortex_driver::SetToolConfigurationResponse)

(defmethod kortex_driver::SetToolConfigurationRequest
  (:response () (instance kortex_driver::SetToolConfigurationResponse :init)))

(setf (get kortex_driver::SetToolConfigurationRequest :md5sum-) "d0f38c8ed77e0310c120105db58e4900")
(setf (get kortex_driver::SetToolConfigurationRequest :datatype-) "kortex_driver/SetToolConfigurationRequest")
(setf (get kortex_driver::SetToolConfigurationRequest :definition-)
      "ToolConfiguration input

================================================================================
MSG: kortex_driver/ToolConfiguration

CartesianTransform tool_transform
float32 tool_mass
ControlConfig_Position tool_mass_center
================================================================================
MSG: kortex_driver/CartesianTransform

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetToolConfigurationResponse :md5sum-) "d0f38c8ed77e0310c120105db58e4900")
(setf (get kortex_driver::SetToolConfigurationResponse :datatype-) "kortex_driver/SetToolConfigurationResponse")
(setf (get kortex_driver::SetToolConfigurationResponse :definition-)
      "ToolConfiguration input

================================================================================
MSG: kortex_driver/ToolConfiguration

CartesianTransform tool_transform
float32 tool_mass
ControlConfig_Position tool_mass_center
================================================================================
MSG: kortex_driver/CartesianTransform

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetToolConfiguration "d0f38c8ed77e0310c120105db58e4900")


