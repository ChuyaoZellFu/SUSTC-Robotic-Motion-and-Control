;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetJointSpeedSoftLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetJointSpeedSoftLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETJOINTSPEEDSOFTLIMITS")
  (make-package "KORTEX_DRIVER::SETJOINTSPEEDSOFTLIMITS"))
(unless (find-package "KORTEX_DRIVER::SETJOINTSPEEDSOFTLIMITSREQUEST")
  (make-package "KORTEX_DRIVER::SETJOINTSPEEDSOFTLIMITSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETJOINTSPEEDSOFTLIMITSRESPONSE")
  (make-package "KORTEX_DRIVER::SETJOINTSPEEDSOFTLIMITSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetJointSpeedSoftLimitsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetJointSpeedSoftLimitsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::JointSpeedSoftLimits :init))
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
    ;; kortex_driver/JointSpeedSoftLimits _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointSpeedSoftLimits _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointSpeedSoftLimits _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetJointSpeedSoftLimitsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetJointSpeedSoftLimitsResponse
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

(defclass kortex_driver::SetJointSpeedSoftLimits
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetJointSpeedSoftLimits :md5sum-) "f67059c3b3849506a74d17610ccdb240")
(setf (get kortex_driver::SetJointSpeedSoftLimits :datatype-) "kortex_driver/SetJointSpeedSoftLimits")
(setf (get kortex_driver::SetJointSpeedSoftLimits :request) kortex_driver::SetJointSpeedSoftLimitsRequest)
(setf (get kortex_driver::SetJointSpeedSoftLimits :response) kortex_driver::SetJointSpeedSoftLimitsResponse)

(defmethod kortex_driver::SetJointSpeedSoftLimitsRequest
  (:response () (instance kortex_driver::SetJointSpeedSoftLimitsResponse :init)))

(setf (get kortex_driver::SetJointSpeedSoftLimitsRequest :md5sum-) "f67059c3b3849506a74d17610ccdb240")
(setf (get kortex_driver::SetJointSpeedSoftLimitsRequest :datatype-) "kortex_driver/SetJointSpeedSoftLimitsRequest")
(setf (get kortex_driver::SetJointSpeedSoftLimitsRequest :definition-)
      "JointSpeedSoftLimits input

================================================================================
MSG: kortex_driver/JointSpeedSoftLimits

uint32 control_mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetJointSpeedSoftLimitsResponse :md5sum-) "f67059c3b3849506a74d17610ccdb240")
(setf (get kortex_driver::SetJointSpeedSoftLimitsResponse :datatype-) "kortex_driver/SetJointSpeedSoftLimitsResponse")
(setf (get kortex_driver::SetJointSpeedSoftLimitsResponse :definition-)
      "JointSpeedSoftLimits input

================================================================================
MSG: kortex_driver/JointSpeedSoftLimits

uint32 control_mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetJointSpeedSoftLimits "f67059c3b3849506a74d17610ccdb240")


