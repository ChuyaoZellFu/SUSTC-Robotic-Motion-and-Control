;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetKinematicSoftLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetKinematicSoftLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETKINEMATICSOFTLIMITS")
  (make-package "KORTEX_DRIVER::GETKINEMATICSOFTLIMITS"))
(unless (find-package "KORTEX_DRIVER::GETKINEMATICSOFTLIMITSREQUEST")
  (make-package "KORTEX_DRIVER::GETKINEMATICSOFTLIMITSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETKINEMATICSOFTLIMITSRESPONSE")
  (make-package "KORTEX_DRIVER::GETKINEMATICSOFTLIMITSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetKinematicSoftLimitsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetKinematicSoftLimitsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControlConfig_ControlModeInformation :init))
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
    ;; kortex_driver/ControlConfig_ControlModeInformation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControlConfig_ControlModeInformation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControlConfig_ControlModeInformation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetKinematicSoftLimitsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetKinematicSoftLimitsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::KinematicLimits :init))
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
    ;; kortex_driver/KinematicLimits _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/KinematicLimits _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/KinematicLimits _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetKinematicSoftLimits
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetKinematicSoftLimits :md5sum-) "8372b01bbf6aa0f718ce44248decf90a")
(setf (get kortex_driver::GetKinematicSoftLimits :datatype-) "kortex_driver/GetKinematicSoftLimits")
(setf (get kortex_driver::GetKinematicSoftLimits :request) kortex_driver::GetKinematicSoftLimitsRequest)
(setf (get kortex_driver::GetKinematicSoftLimits :response) kortex_driver::GetKinematicSoftLimitsResponse)

(defmethod kortex_driver::GetKinematicSoftLimitsRequest
  (:response () (instance kortex_driver::GetKinematicSoftLimitsResponse :init)))

(setf (get kortex_driver::GetKinematicSoftLimitsRequest :md5sum-) "8372b01bbf6aa0f718ce44248decf90a")
(setf (get kortex_driver::GetKinematicSoftLimitsRequest :datatype-) "kortex_driver/GetKinematicSoftLimitsRequest")
(setf (get kortex_driver::GetKinematicSoftLimitsRequest :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
KinematicLimits output

================================================================================
MSG: kortex_driver/KinematicLimits

uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
")

(setf (get kortex_driver::GetKinematicSoftLimitsResponse :md5sum-) "8372b01bbf6aa0f718ce44248decf90a")
(setf (get kortex_driver::GetKinematicSoftLimitsResponse :datatype-) "kortex_driver/GetKinematicSoftLimitsResponse")
(setf (get kortex_driver::GetKinematicSoftLimitsResponse :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
KinematicLimits output

================================================================================
MSG: kortex_driver/KinematicLimits

uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
")



(provide :kortex_driver/GetKinematicSoftLimits "8372b01bbf6aa0f718ce44248decf90a")


