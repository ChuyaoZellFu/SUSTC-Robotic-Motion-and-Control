;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlaySelectedJointTrajectory)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlaySelectedJointTrajectory (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYSELECTEDJOINTTRAJECTORY")
  (make-package "KORTEX_DRIVER::PLAYSELECTEDJOINTTRAJECTORY"))
(unless (find-package "KORTEX_DRIVER::PLAYSELECTEDJOINTTRAJECTORYREQUEST")
  (make-package "KORTEX_DRIVER::PLAYSELECTEDJOINTTRAJECTORYREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYSELECTEDJOINTTRAJECTORYRESPONSE")
  (make-package "KORTEX_DRIVER::PLAYSELECTEDJOINTTRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlaySelectedJointTrajectoryRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlaySelectedJointTrajectoryRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ConstrainedJointAngle :init))
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
    ;; kortex_driver/ConstrainedJointAngle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ConstrainedJointAngle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ConstrainedJointAngle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlaySelectedJointTrajectoryResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlaySelectedJointTrajectoryResponse
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

(defclass kortex_driver::PlaySelectedJointTrajectory
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlaySelectedJointTrajectory :md5sum-) "83e0097177e42b394a0d659ed427f041")
(setf (get kortex_driver::PlaySelectedJointTrajectory :datatype-) "kortex_driver/PlaySelectedJointTrajectory")
(setf (get kortex_driver::PlaySelectedJointTrajectory :request) kortex_driver::PlaySelectedJointTrajectoryRequest)
(setf (get kortex_driver::PlaySelectedJointTrajectory :response) kortex_driver::PlaySelectedJointTrajectoryResponse)

(defmethod kortex_driver::PlaySelectedJointTrajectoryRequest
  (:response () (instance kortex_driver::PlaySelectedJointTrajectoryResponse :init)))

(setf (get kortex_driver::PlaySelectedJointTrajectoryRequest :md5sum-) "83e0097177e42b394a0d659ed427f041")
(setf (get kortex_driver::PlaySelectedJointTrajectoryRequest :datatype-) "kortex_driver/PlaySelectedJointTrajectoryRequest")
(setf (get kortex_driver::PlaySelectedJointTrajectoryRequest :definition-)
      "ConstrainedJointAngle input

================================================================================
MSG: kortex_driver/ConstrainedJointAngle

uint32 joint_identifier
float32 value
JointTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/JointTrajectoryConstraint

uint32 type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PlaySelectedJointTrajectoryResponse :md5sum-) "83e0097177e42b394a0d659ed427f041")
(setf (get kortex_driver::PlaySelectedJointTrajectoryResponse :datatype-) "kortex_driver/PlaySelectedJointTrajectoryResponse")
(setf (get kortex_driver::PlaySelectedJointTrajectoryResponse :definition-)
      "ConstrainedJointAngle input

================================================================================
MSG: kortex_driver/ConstrainedJointAngle

uint32 joint_identifier
float32 value
JointTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/JointTrajectoryConstraint

uint32 type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PlaySelectedJointTrajectory "83e0097177e42b394a0d659ed427f041")


