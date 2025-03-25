;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetToolConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetToolConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETTOOLCONFIGURATION")
  (make-package "KORTEX_DRIVER::RESETTOOLCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::RESETTOOLCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::RESETTOOLCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETTOOLCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::RESETTOOLCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetToolConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetToolConfigurationRequest
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

(defclass kortex_driver::ResetToolConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetToolConfigurationResponse
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

(defclass kortex_driver::ResetToolConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetToolConfiguration :md5sum-) "49c06fd31c2188ee0d629d1b8491b1f0")
(setf (get kortex_driver::ResetToolConfiguration :datatype-) "kortex_driver/ResetToolConfiguration")
(setf (get kortex_driver::ResetToolConfiguration :request) kortex_driver::ResetToolConfigurationRequest)
(setf (get kortex_driver::ResetToolConfiguration :response) kortex_driver::ResetToolConfigurationResponse)

(defmethod kortex_driver::ResetToolConfigurationRequest
  (:response () (instance kortex_driver::ResetToolConfigurationResponse :init)))

(setf (get kortex_driver::ResetToolConfigurationRequest :md5sum-) "49c06fd31c2188ee0d629d1b8491b1f0")
(setf (get kortex_driver::ResetToolConfigurationRequest :datatype-) "kortex_driver/ResetToolConfigurationRequest")
(setf (get kortex_driver::ResetToolConfigurationRequest :definition-)
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

(setf (get kortex_driver::ResetToolConfigurationResponse :md5sum-) "49c06fd31c2188ee0d629d1b8491b1f0")
(setf (get kortex_driver::ResetToolConfigurationResponse :datatype-) "kortex_driver/ResetToolConfigurationResponse")
(setf (get kortex_driver::ResetToolConfigurationResponse :definition-)
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



(provide :kortex_driver/ResetToolConfiguration "49c06fd31c2188ee0d629d1b8491b1f0")


