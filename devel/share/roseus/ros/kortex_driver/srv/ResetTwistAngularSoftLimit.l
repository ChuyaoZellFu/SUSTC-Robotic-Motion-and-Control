;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetTwistAngularSoftLimit)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetTwistAngularSoftLimit (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETTWISTANGULARSOFTLIMIT")
  (make-package "KORTEX_DRIVER::RESETTWISTANGULARSOFTLIMIT"))
(unless (find-package "KORTEX_DRIVER::RESETTWISTANGULARSOFTLIMITREQUEST")
  (make-package "KORTEX_DRIVER::RESETTWISTANGULARSOFTLIMITREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETTWISTANGULARSOFTLIMITRESPONSE")
  (make-package "KORTEX_DRIVER::RESETTWISTANGULARSOFTLIMITRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetTwistAngularSoftLimitRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetTwistAngularSoftLimitRequest
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

(defclass kortex_driver::ResetTwistAngularSoftLimitResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetTwistAngularSoftLimitResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::TwistAngularSoftLimit :init))
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
    ;; kortex_driver/TwistAngularSoftLimit _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TwistAngularSoftLimit _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TwistAngularSoftLimit _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ResetTwistAngularSoftLimit
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetTwistAngularSoftLimit :md5sum-) "24d97d0cfe0dd3c18032abd06a33780b")
(setf (get kortex_driver::ResetTwistAngularSoftLimit :datatype-) "kortex_driver/ResetTwistAngularSoftLimit")
(setf (get kortex_driver::ResetTwistAngularSoftLimit :request) kortex_driver::ResetTwistAngularSoftLimitRequest)
(setf (get kortex_driver::ResetTwistAngularSoftLimit :response) kortex_driver::ResetTwistAngularSoftLimitResponse)

(defmethod kortex_driver::ResetTwistAngularSoftLimitRequest
  (:response () (instance kortex_driver::ResetTwistAngularSoftLimitResponse :init)))

(setf (get kortex_driver::ResetTwistAngularSoftLimitRequest :md5sum-) "24d97d0cfe0dd3c18032abd06a33780b")
(setf (get kortex_driver::ResetTwistAngularSoftLimitRequest :datatype-) "kortex_driver/ResetTwistAngularSoftLimitRequest")
(setf (get kortex_driver::ResetTwistAngularSoftLimitRequest :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
TwistAngularSoftLimit output

================================================================================
MSG: kortex_driver/TwistAngularSoftLimit

uint32 control_mode
")

(setf (get kortex_driver::ResetTwistAngularSoftLimitResponse :md5sum-) "24d97d0cfe0dd3c18032abd06a33780b")
(setf (get kortex_driver::ResetTwistAngularSoftLimitResponse :datatype-) "kortex_driver/ResetTwistAngularSoftLimitResponse")
(setf (get kortex_driver::ResetTwistAngularSoftLimitResponse :definition-)
      "ControlConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ControlConfig_ControlModeInformation

---
TwistAngularSoftLimit output

================================================================================
MSG: kortex_driver/TwistAngularSoftLimit

uint32 control_mode
")



(provide :kortex_driver/ResetTwistAngularSoftLimit "24d97d0cfe0dd3c18032abd06a33780b")


