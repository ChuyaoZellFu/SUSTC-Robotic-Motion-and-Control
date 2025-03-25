;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ComputeForwardKinematics)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ComputeForwardKinematics (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COMPUTEFORWARDKINEMATICS")
  (make-package "KORTEX_DRIVER::COMPUTEFORWARDKINEMATICS"))
(unless (find-package "KORTEX_DRIVER::COMPUTEFORWARDKINEMATICSREQUEST")
  (make-package "KORTEX_DRIVER::COMPUTEFORWARDKINEMATICSREQUEST"))
(unless (find-package "KORTEX_DRIVER::COMPUTEFORWARDKINEMATICSRESPONSE")
  (make-package "KORTEX_DRIVER::COMPUTEFORWARDKINEMATICSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ComputeForwardKinematicsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ComputeForwardKinematicsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::JointAngles :init))
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
    ;; kortex_driver/JointAngles _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointAngles _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointAngles _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ComputeForwardKinematicsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ComputeForwardKinematicsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Pose :init))
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
    ;; kortex_driver/Pose _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Pose _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Pose _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ComputeForwardKinematics
  :super ros::object
  :slots ())

(setf (get kortex_driver::ComputeForwardKinematics :md5sum-) "2406133d7d6bcd723f8f11457d1f2636")
(setf (get kortex_driver::ComputeForwardKinematics :datatype-) "kortex_driver/ComputeForwardKinematics")
(setf (get kortex_driver::ComputeForwardKinematics :request) kortex_driver::ComputeForwardKinematicsRequest)
(setf (get kortex_driver::ComputeForwardKinematics :response) kortex_driver::ComputeForwardKinematicsResponse)

(defmethod kortex_driver::ComputeForwardKinematicsRequest
  (:response () (instance kortex_driver::ComputeForwardKinematicsResponse :init)))

(setf (get kortex_driver::ComputeForwardKinematicsRequest :md5sum-) "2406133d7d6bcd723f8f11457d1f2636")
(setf (get kortex_driver::ComputeForwardKinematicsRequest :datatype-) "kortex_driver/ComputeForwardKinematicsRequest")
(setf (get kortex_driver::ComputeForwardKinematicsRequest :definition-)
      "JointAngles input

================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
---
Pose output

================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
")

(setf (get kortex_driver::ComputeForwardKinematicsResponse :md5sum-) "2406133d7d6bcd723f8f11457d1f2636")
(setf (get kortex_driver::ComputeForwardKinematicsResponse :datatype-) "kortex_driver/ComputeForwardKinematicsResponse")
(setf (get kortex_driver::ComputeForwardKinematicsResponse :definition-)
      "JointAngles input

================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
---
Pose output

================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
")



(provide :kortex_driver/ComputeForwardKinematics "2406133d7d6bcd723f8f11457d1f2636")


