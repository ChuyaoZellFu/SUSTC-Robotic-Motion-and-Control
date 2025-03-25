;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllJointsTorqueHardLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllJointsTorqueHardLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSTORQUEHARDLIMITATION")
  (make-package "KORTEX_DRIVER::GETALLJOINTSTORQUEHARDLIMITATION"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSTORQUEHARDLIMITATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETALLJOINTSTORQUEHARDLIMITATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSTORQUEHARDLIMITATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLJOINTSTORQUEHARDLIMITATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllJointsTorqueHardLimitationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllJointsTorqueHardLimitationRequest
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

(defclass kortex_driver::GetAllJointsTorqueHardLimitationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllJointsTorqueHardLimitationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::JointsLimitationsList :init))
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
    ;; kortex_driver/JointsLimitationsList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointsLimitationsList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointsLimitationsList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllJointsTorqueHardLimitation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllJointsTorqueHardLimitation :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsTorqueHardLimitation :datatype-) "kortex_driver/GetAllJointsTorqueHardLimitation")
(setf (get kortex_driver::GetAllJointsTorqueHardLimitation :request) kortex_driver::GetAllJointsTorqueHardLimitationRequest)
(setf (get kortex_driver::GetAllJointsTorqueHardLimitation :response) kortex_driver::GetAllJointsTorqueHardLimitationResponse)

(defmethod kortex_driver::GetAllJointsTorqueHardLimitationRequest
  (:response () (instance kortex_driver::GetAllJointsTorqueHardLimitationResponse :init)))

(setf (get kortex_driver::GetAllJointsTorqueHardLimitationRequest :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsTorqueHardLimitationRequest :datatype-) "kortex_driver/GetAllJointsTorqueHardLimitationRequest")
(setf (get kortex_driver::GetAllJointsTorqueHardLimitationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
JointsLimitationsList output

================================================================================
MSG: kortex_driver/JointsLimitationsList

JointLimitation[] joints_limitations
================================================================================
MSG: kortex_driver/JointLimitation

uint32 joint_identifier
uint32 type
")

(setf (get kortex_driver::GetAllJointsTorqueHardLimitationResponse :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsTorqueHardLimitationResponse :datatype-) "kortex_driver/GetAllJointsTorqueHardLimitationResponse")
(setf (get kortex_driver::GetAllJointsTorqueHardLimitationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
JointsLimitationsList output

================================================================================
MSG: kortex_driver/JointsLimitationsList

JointLimitation[] joints_limitations
================================================================================
MSG: kortex_driver/JointLimitation

uint32 joint_identifier
uint32 type
")



(provide :kortex_driver/GetAllJointsTorqueHardLimitation "80ab5247f79a646a096a6bb5bc451a7d")


