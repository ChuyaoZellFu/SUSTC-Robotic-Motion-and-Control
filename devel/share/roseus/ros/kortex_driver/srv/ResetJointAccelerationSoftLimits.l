;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetJointAccelerationSoftLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetJointAccelerationSoftLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETJOINTACCELERATIONSOFTLIMITS")
  (make-package "KORTEX_DRIVER::RESETJOINTACCELERATIONSOFTLIMITS"))
(unless (find-package "KORTEX_DRIVER::RESETJOINTACCELERATIONSOFTLIMITSREQUEST")
  (make-package "KORTEX_DRIVER::RESETJOINTACCELERATIONSOFTLIMITSREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETJOINTACCELERATIONSOFTLIMITSRESPONSE")
  (make-package "KORTEX_DRIVER::RESETJOINTACCELERATIONSOFTLIMITSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetJointAccelerationSoftLimitsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetJointAccelerationSoftLimitsRequest
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

(defclass kortex_driver::ResetJointAccelerationSoftLimitsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetJointAccelerationSoftLimitsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::JointAccelerationSoftLimits :init))
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
    ;; kortex_driver/JointAccelerationSoftLimits _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointAccelerationSoftLimits _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointAccelerationSoftLimits _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ResetJointAccelerationSoftLimits
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetJointAccelerationSoftLimits :md5sum-) "6dba409e37ab85922f7777c936234d59")
(setf (get kortex_driver::ResetJointAccelerationSoftLimits :datatype-) "kortex_driver/ResetJointAccelerationSoftLimits")
(setf (get kortex_driver::ResetJointAccelerationSoftLimits :request) kortex_driver::ResetJointAccelerationSoftLimitsRequest)
(setf (get kortex_driver::ResetJointAccelerationSoftLimits :response) kortex_driver::ResetJointAccelerationSoftLimitsResponse)

(defmethod kortex_driver::ResetJointAccelerationSoftLimitsRequest
  (:response () (instance kortex_driver::ResetJointAccelerationSoftLimitsResponse :init)))

(setf (get kortex_driver::ResetJointAccelerationSoftLimitsRequest :md5sum-) "6dba409e37ab85922f7777c936234d59")
(setf (get kortex_driver::ResetJointAccelerationSoftLimitsRequest :datatype-) "kortex_driver/ResetJointAccelerationSoftLimitsRequest")
(setf (get kortex_driver::ResetJointAccelerationSoftLimitsRequest :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
JointAccelerationSoftLimits output

================================================================================
MSG: kortex_driver/JointAccelerationSoftLimits

uint32 control_mode
")

(setf (get kortex_driver::ResetJointAccelerationSoftLimitsResponse :md5sum-) "6dba409e37ab85922f7777c936234d59")
(setf (get kortex_driver::ResetJointAccelerationSoftLimitsResponse :datatype-) "kortex_driver/ResetJointAccelerationSoftLimitsResponse")
(setf (get kortex_driver::ResetJointAccelerationSoftLimitsResponse :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
JointAccelerationSoftLimits output

================================================================================
MSG: kortex_driver/JointAccelerationSoftLimits

uint32 control_mode
")



(provide :kortex_driver/ResetJointAccelerationSoftLimits "6dba409e37ab85922f7777c936234d59")


