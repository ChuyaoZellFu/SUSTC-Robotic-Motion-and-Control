;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllJointsSpeedSoftLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllJointsSpeedSoftLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSSPEEDSOFTLIMITATION")
  (make-package "KORTEX_DRIVER::GETALLJOINTSSPEEDSOFTLIMITATION"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSSPEEDSOFTLIMITATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETALLJOINTSSPEEDSOFTLIMITATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLJOINTSSPEEDSOFTLIMITATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLJOINTSSPEEDSOFTLIMITATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllJointsSpeedSoftLimitationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllJointsSpeedSoftLimitationRequest
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

(defclass kortex_driver::GetAllJointsSpeedSoftLimitationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllJointsSpeedSoftLimitationResponse
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

(defclass kortex_driver::GetAllJointsSpeedSoftLimitation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllJointsSpeedSoftLimitation :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsSpeedSoftLimitation :datatype-) "kortex_driver/GetAllJointsSpeedSoftLimitation")
(setf (get kortex_driver::GetAllJointsSpeedSoftLimitation :request) kortex_driver::GetAllJointsSpeedSoftLimitationRequest)
(setf (get kortex_driver::GetAllJointsSpeedSoftLimitation :response) kortex_driver::GetAllJointsSpeedSoftLimitationResponse)

(defmethod kortex_driver::GetAllJointsSpeedSoftLimitationRequest
  (:response () (instance kortex_driver::GetAllJointsSpeedSoftLimitationResponse :init)))

(setf (get kortex_driver::GetAllJointsSpeedSoftLimitationRequest :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsSpeedSoftLimitationRequest :datatype-) "kortex_driver/GetAllJointsSpeedSoftLimitationRequest")
(setf (get kortex_driver::GetAllJointsSpeedSoftLimitationRequest :definition-)
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

(setf (get kortex_driver::GetAllJointsSpeedSoftLimitationResponse :md5sum-) "80ab5247f79a646a096a6bb5bc451a7d")
(setf (get kortex_driver::GetAllJointsSpeedSoftLimitationResponse :datatype-) "kortex_driver/GetAllJointsSpeedSoftLimitationResponse")
(setf (get kortex_driver::GetAllJointsSpeedSoftLimitationResponse :definition-)
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



(provide :kortex_driver/GetAllJointsSpeedSoftLimitation "80ab5247f79a646a096a6bb5bc451a7d")


