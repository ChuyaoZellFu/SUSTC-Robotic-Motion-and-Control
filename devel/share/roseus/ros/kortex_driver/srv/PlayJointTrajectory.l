;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlayJointTrajectory)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlayJointTrajectory (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYJOINTTRAJECTORY")
  (make-package "KORTEX_DRIVER::PLAYJOINTTRAJECTORY"))
(unless (find-package "KORTEX_DRIVER::PLAYJOINTTRAJECTORYREQUEST")
  (make-package "KORTEX_DRIVER::PLAYJOINTTRAJECTORYREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYJOINTTRAJECTORYRESPONSE")
  (make-package "KORTEX_DRIVER::PLAYJOINTTRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlayJointTrajectoryRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlayJointTrajectoryRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ConstrainedJointAngles :init))
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
    ;; kortex_driver/ConstrainedJointAngles _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ConstrainedJointAngles _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ConstrainedJointAngles _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlayJointTrajectoryResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlayJointTrajectoryResponse
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

(defclass kortex_driver::PlayJointTrajectory
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlayJointTrajectory :md5sum-) "f04458839c6d778db05eb68c1fd72ac4")
(setf (get kortex_driver::PlayJointTrajectory :datatype-) "kortex_driver/PlayJointTrajectory")
(setf (get kortex_driver::PlayJointTrajectory :request) kortex_driver::PlayJointTrajectoryRequest)
(setf (get kortex_driver::PlayJointTrajectory :response) kortex_driver::PlayJointTrajectoryResponse)

(defmethod kortex_driver::PlayJointTrajectoryRequest
  (:response () (instance kortex_driver::PlayJointTrajectoryResponse :init)))

(setf (get kortex_driver::PlayJointTrajectoryRequest :md5sum-) "f04458839c6d778db05eb68c1fd72ac4")
(setf (get kortex_driver::PlayJointTrajectoryRequest :datatype-) "kortex_driver/PlayJointTrajectoryRequest")
(setf (get kortex_driver::PlayJointTrajectoryRequest :definition-)
      "ConstrainedJointAngles input

================================================================================
MSG: kortex_driver/ConstrainedJointAngles

JointAngles joint_angles
JointTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
float32 value
================================================================================
MSG: kortex_driver/JointTrajectoryConstraint

uint32 type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PlayJointTrajectoryResponse :md5sum-) "f04458839c6d778db05eb68c1fd72ac4")
(setf (get kortex_driver::PlayJointTrajectoryResponse :datatype-) "kortex_driver/PlayJointTrajectoryResponse")
(setf (get kortex_driver::PlayJointTrajectoryResponse :definition-)
      "ConstrainedJointAngles input

================================================================================
MSG: kortex_driver/ConstrainedJointAngles

JointAngles joint_angles
JointTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
float32 value
================================================================================
MSG: kortex_driver/JointTrajectoryConstraint

uint32 type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PlayJointTrajectory "f04458839c6d778db05eb68c1fd72ac4")


