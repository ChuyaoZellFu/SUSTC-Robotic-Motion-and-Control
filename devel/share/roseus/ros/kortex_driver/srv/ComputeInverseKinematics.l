;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ComputeInverseKinematics)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ComputeInverseKinematics (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COMPUTEINVERSEKINEMATICS")
  (make-package "KORTEX_DRIVER::COMPUTEINVERSEKINEMATICS"))
(unless (find-package "KORTEX_DRIVER::COMPUTEINVERSEKINEMATICSREQUEST")
  (make-package "KORTEX_DRIVER::COMPUTEINVERSEKINEMATICSREQUEST"))
(unless (find-package "KORTEX_DRIVER::COMPUTEINVERSEKINEMATICSRESPONSE")
  (make-package "KORTEX_DRIVER::COMPUTEINVERSEKINEMATICSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ComputeInverseKinematicsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ComputeInverseKinematicsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::IKData :init))
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
    ;; kortex_driver/IKData _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/IKData _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/IKData _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ComputeInverseKinematicsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ComputeInverseKinematicsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::JointAngles :init))
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
    ;; kortex_driver/JointAngles _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointAngles _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointAngles _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ComputeInverseKinematics
  :super ros::object
  :slots ())

(setf (get kortex_driver::ComputeInverseKinematics :md5sum-) "290825a1e5de199dc18075d261a5fee3")
(setf (get kortex_driver::ComputeInverseKinematics :datatype-) "kortex_driver/ComputeInverseKinematics")
(setf (get kortex_driver::ComputeInverseKinematics :request) kortex_driver::ComputeInverseKinematicsRequest)
(setf (get kortex_driver::ComputeInverseKinematics :response) kortex_driver::ComputeInverseKinematicsResponse)

(defmethod kortex_driver::ComputeInverseKinematicsRequest
  (:response () (instance kortex_driver::ComputeInverseKinematicsResponse :init)))

(setf (get kortex_driver::ComputeInverseKinematicsRequest :md5sum-) "290825a1e5de199dc18075d261a5fee3")
(setf (get kortex_driver::ComputeInverseKinematicsRequest :datatype-) "kortex_driver/ComputeInverseKinematicsRequest")
(setf (get kortex_driver::ComputeInverseKinematicsRequest :definition-)
      "IKData input

================================================================================
MSG: kortex_driver/IKData

Pose cartesian_pose
JointAngles guess
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
---
JointAngles output

================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
")

(setf (get kortex_driver::ComputeInverseKinematicsResponse :md5sum-) "290825a1e5de199dc18075d261a5fee3")
(setf (get kortex_driver::ComputeInverseKinematicsResponse :datatype-) "kortex_driver/ComputeInverseKinematicsResponse")
(setf (get kortex_driver::ComputeInverseKinematicsResponse :definition-)
      "IKData input

================================================================================
MSG: kortex_driver/IKData

Pose cartesian_pose
JointAngles guess
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
---
JointAngles output

================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
")



(provide :kortex_driver/ComputeInverseKinematics "290825a1e5de199dc18075d261a5fee3")


