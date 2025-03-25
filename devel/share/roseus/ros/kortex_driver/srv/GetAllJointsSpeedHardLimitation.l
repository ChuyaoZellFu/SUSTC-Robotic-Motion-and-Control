;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllJointsSpeedHardLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllJointsSpeedHardLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSSPEEDHARDLIMITATION")
  (make-package "KORTEX_DRIVER::GETALLJOINTSSPEEDHARDLIMITATION"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSSPEEDHARDLIMITATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETALLJOINTSSPEEDHARDLIMITATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSSPEEDHARDLIMITATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLJOINTSSPEEDHARDLIMITATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllJointsSpeedHardLimitationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllJointsSpeedHardLimitationRequest
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

(defclass kortex_driver::GetAllJointsSpeedHardLimitationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllJointsSpeedHardLimitationResponse
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

(defclass kortex_driver::GetAllJointsSpeedHardLimitation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllJointsSpeedHardLimitation :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsSpeedHardLimitation :datatype-) "kortex_driver/GetAllJointsSpeedHardLimitation")
(setf (get kortex_driver::GetAllJointsSpeedHardLimitation :request) kortex_driver::GetAllJointsSpeedHardLimitationRequest)
(setf (get kortex_driver::GetAllJointsSpeedHardLimitation :response) kortex_driver::GetAllJointsSpeedHardLimitationResponse)

(defmethod kortex_driver::GetAllJointsSpeedHardLimitationRequest
  (:response () (instance kortex_driver::GetAllJointsSpeedHardLimitationResponse :init)))

(setf (get kortex_driver::GetAllJointsSpeedHardLimitationRequest :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsSpeedHardLimitationRequest :datatype-) "kortex_driver/GetAllJointsSpeedHardLimitationRequest")
(setf (get kortex_driver::GetAllJointsSpeedHardLimitationRequest :definition-)
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

(setf (get kortex_driver::GetAllJointsSpeedHardLimitationResponse :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsSpeedHardLimitationResponse :datatype-) "kortex_driver/GetAllJointsSpeedHardLimitationResponse")
(setf (get kortex_driver::GetAllJointsSpeedHardLimitationResponse :definition-)
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



(provide :kortex_driver/GetAllJointsSpeedHardLimitation "80ab5247f79a646a096a6bb5bc451a7d")


