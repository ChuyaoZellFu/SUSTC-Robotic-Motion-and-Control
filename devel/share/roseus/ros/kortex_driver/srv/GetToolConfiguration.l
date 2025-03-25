;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetToolConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetToolConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETTOOLCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETTOOLCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETTOOLCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETTOOLCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETTOOLCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETTOOLCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetToolConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetToolConfigurationRequest
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

(defclass kortex_driver::GetToolConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetToolConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ToolConfiguration :init))
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
    ;; kortex_driver/ToolConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ToolConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ToolConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetToolConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetToolConfiguration :md5sum-) "49c06fd31c2188ee0d629d1b8491b1f0")
(setf (get kortex_driver::GetToolConfiguration :datatype-) "kortex_driver/GetToolConfiguration")
(setf (get kortex_driver::GetToolConfiguration :request) kortex_driver::GetToolConfigurationRequest)
(setf (get kortex_driver::GetToolConfiguration :response) kortex_driver::GetToolConfigurationResponse)

(defmethod kortex_driver::GetToolConfigurationRequest
  (:response () (instance kortex_driver::GetToolConfigurationResponse :init)))

(setf (get kortex_driver::GetToolConfigurationRequest :md5sum-) "49c06fd31c2188ee0d629d1b8491b1f0")
(setf (get kortex_driver::GetToolConfigurationRequest :datatype-) "kortex_driver/GetToolConfigurationRequest")
(setf (get kortex_driver::GetToolConfigurationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ToolConfiguration output

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
")

(setf (get kortex_driver::GetToolConfigurationResponse :md5sum-) "49c06fd31c2188ee0d629d1b8491b1f0")
(setf (get kortex_driver::GetToolConfigurationResponse :datatype-) "kortex_driver/GetToolConfigurationResponse")
(setf (get kortex_driver::GetToolConfigurationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ToolConfiguration output

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
")



(provide :kortex_driver/GetToolConfiguration "49c06fd31c2188ee0d629d1b8491b1f0")


