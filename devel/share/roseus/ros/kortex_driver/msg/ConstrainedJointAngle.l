;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConstrainedJointAngle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConstrainedJointAngle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONSTRAINEDJOINTANGLE")
  (make-package "KORTEX_DRIVER::CONSTRAINEDJOINTANGLE"))

(in-package "ROS")
;;//! \htmlinclude ConstrainedJointAngle.msg.html


(defclass kortex_driver::ConstrainedJointAngle
  :super ros::object
  :slots (_joint_identifier _value _constraint ))

(defmethod kortex_driver::ConstrainedJointAngle
  (:init
   (&key
    ((:joint_identifier __joint_identifier) 0)
    ((:value __value) 0.0)
    ((:constraint __constraint) (instance kortex_driver::JointTrajectoryConstraint :init))
    )
   (send-super :init)
   (setq _joint_identifier (round __joint_identifier))
   (setq _value (float __value))
   (setq _constraint __constraint)
   self)
  (:joint_identifier
   (&optional __joint_identifier)
   (if __joint_identifier (setq _joint_identifier __joint_identifier)) _joint_identifier)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
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
    ;; uint32 _joint_identifier
    4
    ;; float32 _value
    4
    ;; kortex_driver/JointTrajectoryConstraint _constraint
    (send _constraint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _joint_identifier
       (write-long _joint_identifier s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; kortex_driver/JointTrajectoryConstraint _constraint
       (send _constraint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _joint_identifier
     (setq _joint_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; kortex_driver/JointTrajectoryConstraint _constraint
     (send _constraint :deserialize buf ptr-) (incf ptr- (send _constraint :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ConstrainedJointAngle :md5sum-) "6fb07b64148f47061948eb6c6ef38e00")
(setf (get kortex_driver::ConstrainedJointAngle :datatype-) "kortex_driver/ConstrainedJointAngle")
(setf (get kortex_driver::ConstrainedJointAngle :definition-)
      "
uint32 joint_identifier
float32 value
JointTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/JointTrajectoryConstraint

uint32 type
float32 value
")



(provide :kortex_driver/ConstrainedJointAngle "6fb07b64148f47061948eb6c6ef38e00")


