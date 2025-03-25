;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConstrainedJointAngles)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConstrainedJointAngles (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONSTRAINEDJOINTANGLES")
  (make-package "KORTEX_DRIVER::CONSTRAINEDJOINTANGLES"))

(in-package "ROS")
;;//! \htmlinclude ConstrainedJointAngles.msg.html


(defclass kortex_driver::ConstrainedJointAngles
  :super ros::object
  :slots (_joint_angles _constraint ))

(defmethod kortex_driver::ConstrainedJointAngles
  (:init
   (&key
    ((:joint_angles __joint_angles) (instance kortex_driver::JointAngles :init))
    ((:constraint __constraint) (instance kortex_driver::JointTrajectoryConstraint :init))
    )
   (send-super :init)
   (setq _joint_angles __joint_angles)
   (setq _constraint __constraint)
   self)
  (:joint_angles
   (&rest __joint_angles)
   (if (keywordp (car __joint_angles))
       (send* _joint_angles __joint_angles)
     (progn
       (if __joint_angles (setq _joint_angles (car __joint_angles)))
       _joint_angles)))
  (:constraint
   (&rest __constraint)
   (if (keywordp (car __constraint))
       (send* _constraint __constraint)
     (progn
       (if __constraint (setq _constraint (car __constraint)))
       _constraint)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/JointAngles _joint_angles
    (send _joint_angles :serialization-length)
    ;; kortex_driver/JointTrajectoryConstraint _constraint
    (send _constraint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointAngles _joint_angles
       (send _joint_angles :serialize s)
     ;; kortex_driver/JointTrajectoryConstraint _constraint
       (send _constraint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointAngles _joint_angles
     (send _joint_angles :deserialize buf ptr-) (incf ptr- (send _joint_angles :serialization-length))
   ;; kortex_driver/JointTrajectoryConstraint _constraint
     (send _constraint :deserialize buf ptr-) (incf ptr- (send _constraint :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ConstrainedJointAngles :md5sum-) "75faad16493b9d4c290ef3b7fb5a8947")
(setf (get kortex_driver::ConstrainedJointAngles :datatype-) "kortex_driver/ConstrainedJointAngles")
(setf (get kortex_driver::ConstrainedJointAngles :definition-)
      "
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
float32 value
")



(provide :kortex_driver/ConstrainedJointAngles "75faad16493b9d4c290ef3b7fb5a8947")


