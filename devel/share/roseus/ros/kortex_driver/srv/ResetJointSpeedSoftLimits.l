;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetJointSpeedSoftLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetJointSpeedSoftLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETJOINTSPEEDSOFTLIMITS")
  (make-package "KORTEX_DRIVER::RESETJOINTSPEEDSOFTLIMITS"))
(unless (find-package "KORTEX_DRIVER::RESETJOINTSPEEDSOFTLIMITSREQUEST")
  (make-package "KORTEX_DRIVER::RESETJOINTSPEEDSOFTLIMITSREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETJOINTSPEEDSOFTLIMITSRESPONSE")
  (make-package "KORTEX_DRIVER::RESETJOINTSPEEDSOFTLIMITSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetJointSpeedSoftLimitsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetJointSpeedSoftLimitsRequest
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

(defclass kortex_driver::ResetJointSpeedSoftLimitsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetJointSpeedSoftLimitsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::JointSpeedSoftLimits :init))
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
    ;; kortex_driver/JointSpeedSoftLimits _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointSpeedSoftLimits _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointSpeedSoftLimits _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ResetJointSpeedSoftLimits
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetJointSpeedSoftLimits :md5sum-) "7d22c334332447e55a639a3764ac0a17")
(setf (get kortex_driver::ResetJointSpeedSoftLimits :datatype-) "kortex_driver/ResetJointSpeedSoftLimits")
(setf (get kortex_driver::ResetJointSpeedSoftLimits :request) kortex_driver::ResetJointSpeedSoftLimitsRequest)
(setf (get kortex_driver::ResetJointSpeedSoftLimits :response) kortex_driver::ResetJointSpeedSoftLimitsResponse)

(defmethod kortex_driver::ResetJointSpeedSoftLimitsRequest
  (:response () (instance kortex_driver::ResetJointSpeedSoftLimitsResponse :init)))

(setf (get kortex_driver::ResetJointSpeedSoftLimitsRequest :md5sum-) "7d22c334332447e55a639a3764ac0a17")
(setf (get kortex_driver::ResetJointSpeedSoftLimitsRequest :datatype-) "kortex_driver/ResetJointSpeedSoftLimitsRequest")
(setf (get kortex_driver::ResetJointSpeedSoftLimitsRequest :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
JointSpeedSoftLimits output

================================================================================
MSG: kortex_driver/JointSpeedSoftLimits

uint32 control_mode
")

(setf (get kortex_driver::ResetJointSpeedSoftLimitsResponse :md5sum-) "7d22c334332447e55a639a3764ac0a17")
(setf (get kortex_driver::ResetJointSpeedSoftLimitsResponse :datatype-) "kortex_driver/ResetJointSpeedSoftLimitsResponse")
(setf (get kortex_driver::ResetJointSpeedSoftLimitsResponse :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
JointSpeedSoftLimits output

================================================================================
MSG: kortex_driver/JointSpeedSoftLimits

uint32 control_mode
")



(provide :kortex_driver/ResetJointSpeedSoftLimits "7d22c334332447e55a639a3764ac0a17")


