;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlayPreComputedJointTrajectory)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlayPreComputedJointTrajectory (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYPRECOMPUTEDJOINTTRAJECTORY")
  (make-package "KORTEX_DRIVER::PLAYPRECOMPUTEDJOINTTRAJECTORY"))
(unless (find-package "KORTEX_DRIVER::PLAYPRECOMPUTEDJOINTTRAJECTORYREQUEST")
  (make-package "KORTEX_DRIVER::PLAYPRECOMPUTEDJOINTTRAJECTORYREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYPRECOMPUTEDJOINTTRAJECTORYRESPONSE")
  (make-package "KORTEX_DRIVER::PLAYPRECOMPUTEDJOINTTRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlayPreComputedJointTrajectoryRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlayPreComputedJointTrajectoryRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::PreComputedJointTrajectory :init))
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
    ;; kortex_driver/PreComputedJointTrajectory _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/PreComputedJointTrajectory _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/PreComputedJointTrajectory _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlayPreComputedJointTrajectoryResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlayPreComputedJointTrajectoryResponse
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

(defclass kortex_driver::PlayPreComputedJointTrajectory
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlayPreComputedJointTrajectory :md5sum-) "50902897eedd6708728c63b8108c9da3")
(setf (get kortex_driver::PlayPreComputedJointTrajectory :datatype-) "kortex_driver/PlayPreComputedJointTrajectory")
(setf (get kortex_driver::PlayPreComputedJointTrajectory :request) kortex_driver::PlayPreComputedJointTrajectoryRequest)
(setf (get kortex_driver::PlayPreComputedJointTrajectory :response) kortex_driver::PlayPreComputedJointTrajectoryResponse)

(defmethod kortex_driver::PlayPreComputedJointTrajectoryRequest
  (:response () (instance kortex_driver::PlayPreComputedJointTrajectoryResponse :init)))

(setf (get kortex_driver::PlayPreComputedJointTrajectoryRequest :md5sum-) "50902897eedd6708728c63b8108c9da3")
(setf (get kortex_driver::PlayPreComputedJointTrajectoryRequest :datatype-) "kortex_driver/PlayPreComputedJointTrajectoryRequest")
(setf (get kortex_driver::PlayPreComputedJointTrajectoryRequest :definition-)
      "PreComputedJointTrajectory input

================================================================================
MSG: kortex_driver/PreComputedJointTrajectory

uint32 mode
PreComputedJointTrajectoryElement[] trajectory_elements
================================================================================
MSG: kortex_driver/PreComputedJointTrajectoryElement

float32[] joint_angles
float32[] joint_speeds
float32[] joint_accelerations
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PlayPreComputedJointTrajectoryResponse :md5sum-) "50902897eedd6708728c63b8108c9da3")
(setf (get kortex_driver::PlayPreComputedJointTrajectoryResponse :datatype-) "kortex_driver/PlayPreComputedJointTrajectoryResponse")
(setf (get kortex_driver::PlayPreComputedJointTrajectoryResponse :definition-)
      "PreComputedJointTrajectory input

================================================================================
MSG: kortex_driver/PreComputedJointTrajectory

uint32 mode
PreComputedJointTrajectoryElement[] trajectory_elements
================================================================================
MSG: kortex_driver/PreComputedJointTrajectoryElement

float32[] joint_angles
float32[] joint_speeds
float32[] joint_accelerations
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PlayPreComputedJointTrajectory "50902897eedd6708728c63b8108c9da3")


