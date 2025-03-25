;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConstrainedOrientation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConstrainedOrientation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONSTRAINEDORIENTATION")
  (make-package "KORTEX_DRIVER::CONSTRAINEDORIENTATION"))

(in-package "ROS")
;;//! \htmlinclude ConstrainedOrientation.msg.html


(defclass kortex_driver::ConstrainedOrientation
  :super ros::object
  :slots (_target_orientation _constraint ))

(defmethod kortex_driver::ConstrainedOrientation
  (:init
   (&key
    ((:target_orientation __target_orientation) (instance kortex_driver::Orientation :init))
    ((:constraint __constraint) (instance kortex_driver::CartesianTrajectoryConstraint :init))
    )
   (send-super :init)
   (setq _target_orientation __target_orientation)
   (setq _constraint __constraint)
   self)
  (:target_orientation
   (&rest __target_orientation)
   (if (keywordp (car __target_orientation))
       (send* _target_orientation __target_orientation)
     (progn
       (if __target_orientation (setq _target_orientation (car __target_orientation)))
       _target_orientation)))
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
    ;; kortex_driver/Orientation _target_orientation
    (send _target_orientation :serialization-length)
    ;; kortex_driver/CartesianTrajectoryConstraint _constraint
    (send _constraint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Orientation _target_orientation
       (send _target_orientation :serialize s)
     ;; kortex_driver/CartesianTrajectoryConstraint _constraint
       (send _constraint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Orientation _target_orientation
     (send _target_orientation :deserialize buf ptr-) (incf ptr- (send _target_orientation :serialization-length))
   ;; kortex_driver/CartesianTrajectoryConstraint _constraint
     (send _constraint :deserialize buf ptr-) (incf ptr- (send _constraint :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ConstrainedOrientation :md5sum-) "15f399eb15570e6ba491b979fdea9cfd")
(setf (get kortex_driver::ConstrainedOrientation :datatype-) "kortex_driver/ConstrainedOrientation")
(setf (get kortex_driver::ConstrainedOrientation :definition-)
      "
Orientation target_orientation
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Orientation

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



(provide :kortex_driver/ConstrainedOrientation "15f399eb15570e6ba491b979fdea9cfd")


