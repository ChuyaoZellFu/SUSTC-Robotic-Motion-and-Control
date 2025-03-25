;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConstrainedPosition)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConstrainedPosition (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONSTRAINEDPOSITION")
  (make-package "KORTEX_DRIVER::CONSTRAINEDPOSITION"))

(in-package "ROS")
;;//! \htmlinclude ConstrainedPosition.msg.html


(defclass kortex_driver::ConstrainedPosition
  :super ros::object
  :slots (_target_position _constraint ))

(defmethod kortex_driver::ConstrainedPosition
  (:init
   (&key
    ((:target_position __target_position) (instance kortex_driver::Base_Position :init))
    ((:constraint __constraint) (instance kortex_driver::CartesianTrajectoryConstraint :init))
    )
   (send-super :init)
   (setq _target_position __target_position)
   (setq _constraint __constraint)
   self)
  (:target_position
   (&rest __target_position)
   (if (keywordp (car __target_position))
       (send* _target_position __target_position)
     (progn
       (if __target_position (setq _target_position (car __target_position)))
       _target_position)))
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
    ;; kortex_driver/Base_Position _target_position
    (send _target_position :serialization-length)
    ;; kortex_driver/CartesianTrajectoryConstraint _constraint
    (send _constraint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_Position _target_position
       (send _target_position :serialize s)
     ;; kortex_driver/CartesianTrajectoryConstraint _constraint
       (send _constraint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_Position _target_position
     (send _target_position :deserialize buf ptr-) (incf ptr- (send _target_position :serialization-length))
   ;; kortex_driver/CartesianTrajectoryConstraint _constraint
     (send _constraint :deserialize buf ptr-) (incf ptr- (send _constraint :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ConstrainedPosition :md5sum-) "4c5833c3b26267ce0142362e034e1ba3")
(setf (get kortex_driver::ConstrainedPosition :datatype-) "kortex_driver/ConstrainedPosition")
(setf (get kortex_driver::ConstrainedPosition :definition-)
      "
Base_Position target_position
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Base_Position

float32 x
float32 y
float32 z
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



(provide :kortex_driver/ConstrainedPosition "4c5833c3b26267ce0142362e034e1ba3")


