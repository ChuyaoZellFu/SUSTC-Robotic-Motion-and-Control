;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlayCartesianTrajectory)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlayCartesianTrajectory (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORY")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORY"))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYREQUEST")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYRESPONSE")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlayCartesianTrajectoryRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlayCartesianTrajectoryRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ConstrainedPose :init))
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
    ;; kortex_driver/ConstrainedPose _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ConstrainedPose _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ConstrainedPose _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlayCartesianTrajectoryResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlayCartesianTrajectoryResponse
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

(defclass kortex_driver::PlayCartesianTrajectory
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlayCartesianTrajectory :md5sum-) "464f528a5fe3cfaa9d04f8e718f01a5c")
(setf (get kortex_driver::PlayCartesianTrajectory :datatype-) "kortex_driver/PlayCartesianTrajectory")
(setf (get kortex_driver::PlayCartesianTrajectory :request) kortex_driver::PlayCartesianTrajectoryRequest)
(setf (get kortex_driver::PlayCartesianTrajectory :response) kortex_driver::PlayCartesianTrajectoryResponse)

(defmethod kortex_driver::PlayCartesianTrajectoryRequest
  (:response () (instance kortex_driver::PlayCartesianTrajectoryResponse :init)))

(setf (get kortex_driver::PlayCartesianTrajectoryRequest :md5sum-) "464f528a5fe3cfaa9d04f8e718f01a5c")
(setf (get kortex_driver::PlayCartesianTrajectoryRequest :datatype-) "kortex_driver/PlayCartesianTrajectoryRequest")
(setf (get kortex_driver::PlayCartesianTrajectoryRequest :definition-)
      "ConstrainedPose input

================================================================================
MSG: kortex_driver/ConstrainedPose

Pose target_pose
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint

CartesianTrajectoryConstraint_type oneof_type
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint_type

CartesianSpeed[] speed
uint32[] duration
================================================================================
MSG: kortex_driver/CartesianSpeed

float32 translation
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PlayCartesianTrajectoryResponse :md5sum-) "464f528a5fe3cfaa9d04f8e718f01a5c")
(setf (get kortex_driver::PlayCartesianTrajectoryResponse :datatype-) "kortex_driver/PlayCartesianTrajectoryResponse")
(setf (get kortex_driver::PlayCartesianTrajectoryResponse :definition-)
      "ConstrainedPose input

================================================================================
MSG: kortex_driver/ConstrainedPose

Pose target_pose
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint

CartesianTrajectoryConstraint_type oneof_type
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint_type

CartesianSpeed[] speed
uint32[] duration
================================================================================
MSG: kortex_driver/CartesianSpeed

float32 translation
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PlayCartesianTrajectory "464f528a5fe3cfaa9d04f8e718f01a5c")


