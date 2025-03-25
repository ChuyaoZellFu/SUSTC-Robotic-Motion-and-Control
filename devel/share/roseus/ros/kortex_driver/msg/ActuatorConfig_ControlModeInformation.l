;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorConfig_ControlModeInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorConfig_ControlModeInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODEINFORMATION")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_CONTROLMODEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude ActuatorConfig_ControlModeInformation.msg.html


(defclass kortex_driver::ActuatorConfig_ControlModeInformation
  :super ros::object
  :slots (_control_mode ))

(defmethod kortex_driver::ActuatorConfig_ControlModeInformation
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:serialization-length
   ()
   (+
    ;; uint32 _control_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _control_mode
       (write-long _control_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _control_mode
     (setq _control_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ActuatorConfig_ControlModeInformation :md5sum-) "c6c18afc4d9609a0ad8ceeb45431dcd5")
(setf (get kortex_driver::ActuatorConfig_ControlModeInformation :datatype-) "kortex_driver/ActuatorConfig_ControlModeInformation")
(setf (get kortex_driver::ActuatorConfig_ControlModeInformation :definition-)
      "
uint32 control_mode
")



(provide :kortex_driver/ActuatorConfig_ControlModeInformation "c6c18afc4d9609a0ad8ceeb45431dcd5")


