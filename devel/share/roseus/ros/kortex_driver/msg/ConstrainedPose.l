;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConstrainedPose)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConstrainedPose (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONSTRAINEDPOSE")
  (make-package "KORTEX_DRIVER::CONSTRAINEDPOSE"))

(in-package "ROS")
;;//! \htmlinclude ConstrainedPose.msg.html


(defclass kortex_driver::ConstrainedPose
  :super ros::object
  :slots (_target_pose _constraint ))

(defmethod kortex_driver::ConstrainedPose
  (:init
   (&key
    ((:target_pose __target_pose) (instance kortex_driver::Pose :init))
    ((:constraint __constraint) (instance kortex_driver::CartesianTrajectoryConstraint :init))
    )
   (send-super :init)
   (setq _target_pose __target_pose)
   (setq _constraint __constraint)
   self)
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
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
    ;; kortex_driver/Pose _target_pose
    (send _target_pose :serialization-length)
    ;; kortex_driver/CartesianTrajectoryConstraint _constraint
    (send _constraint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Pose _target_pose
       (send _target_pose :serialize s)
     ;; kortex_driver/CartesianTrajectoryConstraint _constraint
       (send _constraint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;; kortex_driver/CartesianTrajectoryConstraint _constraint
     (send _constraint :deserialize buf ptr-) (incf ptr- (send _constraint :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ConstrainedPose :md5sum-) "201b7431451c1b450ae81fca8778070f")
(setf (get kortex_driver::ConstrainedPose :datatype-) "kortex_driver/ConstrainedPose")
(setf (get kortex_driver::ConstrainedPose :definition-)
      "
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
float32 orientation
")



(provide :kortex_driver/ConstrainedPose "201b7431451c1b450ae81fca8778070f")


