;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetJointAccelerationSoftLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetJointAccelerationSoftLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETJOINTACCELERATIONSOFTLIMITS")
  (make-package "KORTEX_DRIVER::SETJOINTACCELERATIONSOFTLIMITS"))
(unless (find-package "KORTEX_DRIVER::SETJOINTACCELERATIONSOFTLIMITSREQUEST")
  (make-package "KORTEX_DRIVER::SETJOINTACCELERATIONSOFTLIMITSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETJOINTACCELERATIONSOFTLIMITSRESPONSE")
  (make-package "KORTEX_DRIVER::SETJOINTACCELERATIONSOFTLIMITSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetJointAccelerationSoftLimitsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetJointAccelerationSoftLimitsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::JointAccelerationSoftLimits :init))
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
    ;; kortex_driver/JointAccelerationSoftLimits _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointAccelerationSoftLimits _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointAccelerationSoftLimits _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetJointAccelerationSoftLimitsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetJointAccelerationSoftLimitsResponse
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

(defclass kortex_driver::SetJointAccelerationSoftLimits
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetJointAccelerationSoftLimits :md5sum-) "cee0ecb0e446175f3b5cec2dbc1076e5")
(setf (get kortex_driver::SetJointAccelerationSoftLimits :datatype-) "kortex_driver/SetJointAccelerationSoftLimits")
(setf (get kortex_driver::SetJointAccelerationSoftLimits :request) kortex_driver::SetJointAccelerationSoftLimitsRequest)
(setf (get kortex_driver::SetJointAccelerationSoftLimits :response) kortex_driver::SetJointAccelerationSoftLimitsResponse)

(defmethod kortex_driver::SetJointAccelerationSoftLimitsRequest
  (:response () (instance kortex_driver::SetJointAccelerationSoftLimitsResponse :init)))

(setf (get kortex_driver::SetJointAccelerationSoftLimitsRequest :md5sum-) "cee0ecb0e446175f3b5cec2dbc1076e5")
(setf (get kortex_driver::SetJointAccelerationSoftLimitsRequest :datatype-) "kortex_driver/SetJointAccelerationSoftLimitsRequest")
(setf (get kortex_driver::SetJointAccelerationSoftLimitsRequest :definition-)
      "JointAccelerationSoftLimits input

================================================================================
MSG: kortex_driver/JointAccelerationSoftLimits

uint32 control_mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetJointAccelerationSoftLimitsResponse :md5sum-) "cee0ecb0e446175f3b5cec2dbc1076e5")
(setf (get kortex_driver::SetJointAccelerationSoftLimitsResponse :datatype-) "kortex_driver/SetJointAccelerationSoftLimitsResponse")
(setf (get kortex_driver::SetJointAccelerationSoftLimitsResponse :definition-)
      "JointAccelerationSoftLimits input

================================================================================
MSG: kortex_driver/JointAccelerationSoftLimits

uint32 control_mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetJointAccelerationSoftLimits "cee0ecb0e446175f3b5cec2dbc1076e5")


