;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetTwistLinearSoftLimit)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetTwistLinearSoftLimit (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETTWISTLINEARSOFTLIMIT")
  (make-package "KORTEX_DRIVER::RESETTWISTLINEARSOFTLIMIT"))
(unless (find-package "KORTEX_DRIVER::RESETTWISTLINEARSOFTLIMITREQUEST")
  (make-package "KORTEX_DRIVER::RESETTWISTLINEARSOFTLIMITREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETTWISTLINEARSOFTLIMITRESPONSE")
  (make-package "KORTEX_DRIVER::RESETTWISTLINEARSOFTLIMITRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetTwistLinearSoftLimitRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetTwistLinearSoftLimitRequest
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

(defclass kortex_driver::ResetTwistLinearSoftLimitResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetTwistLinearSoftLimitResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::TwistLinearSoftLimit :init))
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
    ;; kortex_driver/TwistLinearSoftLimit _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TwistLinearSoftLimit _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TwistLinearSoftLimit _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ResetTwistLinearSoftLimit
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetTwistLinearSoftLimit :md5sum-) "277326cfb7b831ac01de5d007b623790")
(setf (get kortex_driver::ResetTwistLinearSoftLimit :datatype-) "kortex_driver/ResetTwistLinearSoftLimit")
(setf (get kortex_driver::ResetTwistLinearSoftLimit :request) kortex_driver::ResetTwistLinearSoftLimitRequest)
(setf (get kortex_driver::ResetTwistLinearSoftLimit :response) kortex_driver::ResetTwistLinearSoftLimitResponse)

(defmethod kortex_driver::ResetTwistLinearSoftLimitRequest
  (:response () (instance kortex_driver::ResetTwistLinearSoftLimitResponse :init)))

(setf (get kortex_driver::ResetTwistLinearSoftLimitRequest :md5sum-) "277326cfb7b831ac01de5d007b623790")
(setf (get kortex_driver::ResetTwistLinearSoftLimitRequest :datatype-) "kortex_driver/ResetTwistLinearSoftLimitRequest")
(setf (get kortex_driver::ResetTwistLinearSoftLimitRequest :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
TwistLinearSoftLimit output

================================================================================
MSG: kortex_driver/TwistLinearSoftLimit

uint32 control_mode
")

(setf (get kortex_driver::ResetTwistLinearSoftLimitResponse :md5sum-) "277326cfb7b831ac01de5d007b623790")
(setf (get kortex_driver::ResetTwistLinearSoftLimitResponse :datatype-) "kortex_driver/ResetTwistLinearSoftLimitResponse")
(setf (get kortex_driver::ResetTwistLinearSoftLimitResponse :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
TwistLinearSoftLimit output

================================================================================
MSG: kortex_driver/TwistLinearSoftLimit

uint32 control_mode
")



(provide :kortex_driver/ResetTwistLinearSoftLimit "277326cfb7b831ac01de5d007b623790")


