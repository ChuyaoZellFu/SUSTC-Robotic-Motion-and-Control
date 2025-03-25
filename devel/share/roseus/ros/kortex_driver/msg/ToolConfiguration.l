;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ToolConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ToolConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TOOLCONFIGURATION")
  (make-package "KORTEX_DRIVER::TOOLCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude ToolConfiguration.msg.html


(defclass kortex_driver::ToolConfiguration
  :super ros::object
  :slots (_tool_transform _tool_mass _tool_mass_center ))

(defmethod kortex_driver::ToolConfiguration
  (:init
   (&key
    ((:tool_transform __tool_transform) (instance kortex_driver::CartesianTransform :init))
    ((:tool_mass __tool_mass) 0.0)
    ((:tool_mass_center __tool_mass_center) (instance kortex_driver::ControlConfig_Position :init))
    )
   (send-super :init)
   (setq _tool_transform __tool_transform)
   (setq _tool_mass (float __tool_mass))
   (setq _tool_mass_center __tool_mass_center)
   self)
  (:tool_transform
   (&rest __tool_transform)
   (if (keywordp (car __tool_transform))
       (send* _tool_transform __tool_transform)
     (progn
       (if __tool_transform (setq _tool_transform (car __tool_transform)))
       _tool_transform)))
  (:tool_mass
   (&optional __tool_mass)
   (if __tool_mass (setq _tool_mass __tool_mass)) _tool_mass)
  (:tool_mass_center
   (&rest __tool_mass_center)
   (if (keywordp (car __tool_mass_center))
       (send* _tool_mass_center __tool_mass_center)
     (progn
       (if __tool_mass_center (setq _tool_mass_center (car __tool_mass_center)))
       _tool_mass_center)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/CartesianTransform _tool_transform
    (send _tool_transform :serialization-length)
    ;; float32 _tool_mass
    4
    ;; kortex_driver/ControlConfig_Position _tool_mass_center
    (send _tool_mass_center :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CartesianTransform _tool_transform
       (send _tool_transform :serialize s)
     ;; float32 _tool_mass
       (sys::poke _tool_mass (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; kortex_driver/ControlConfig_Position _tool_mass_center
       (send _tool_mass_center :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CartesianTransform _tool_transform
     (send _tool_transform :deserialize buf ptr-) (incf ptr- (send _tool_transform :serialization-length))
   ;; float32 _tool_mass
     (setq _tool_mass (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; kortex_driver/ControlConfig_Position _tool_mass_center
     (send _tool_mass_center :deserialize buf ptr-) (incf ptr- (send _tool_mass_center :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ToolConfiguration :md5sum-) "cf7e6c29cefe7625fb7412c1a3c76941")
(setf (get kortex_driver::ToolConfiguration :datatype-) "kortex_driver/ToolConfiguration")
(setf (get kortex_driver::ToolConfiguration :definition-)
      "
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
float32 z
")



(provide :kortex_driver/ToolConfiguration "cf7e6c29cefe7625fb7412c1a3c76941")


