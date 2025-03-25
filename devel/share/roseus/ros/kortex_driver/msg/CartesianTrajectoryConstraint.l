;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianTrajectoryConstraint)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianTrajectoryConstraint (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANTRAJECTORYCONSTRAINT")
  (make-package "KORTEX_DRIVER::CARTESIANTRAJECTORYCONSTRAINT"))

(in-package "ROS")
;;//! \htmlinclude CartesianTrajectoryConstraint.msg.html


(defclass kortex_driver::CartesianTrajectoryConstraint
  :super ros::object
  :slots (_oneof_type ))

(defmethod kortex_driver::CartesianTrajectoryConstraint
  (:init
   (&key
    ((:oneof_type __oneof_type) (instance kortex_driver::CartesianTrajectoryConstraint_type :init))
    )
   (send-super :init)
   (setq _oneof_type __oneof_type)
   self)
  (:oneof_type
   (&rest __oneof_type)
   (if (keywordp (car __oneof_type))
       (send* _oneof_type __oneof_type)
     (progn
       (if __oneof_type (setq _oneof_type (car __oneof_type)))
       _oneof_type)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/CartesianTrajectoryConstraint_type _oneof_type
    (send _oneof_type :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CartesianTrajectoryConstraint_type _oneof_type
       (send _oneof_type :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CartesianTrajectoryConstraint_type _oneof_type
     (send _oneof_type :deserialize buf ptr-) (incf ptr- (send _oneof_type :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::CartesianTrajectoryConstraint :md5sum-) "4758c371e633c2206a71f0c6833f6d5a")
(setf (get kortex_driver::CartesianTrajectoryConstraint :datatype-) "kortex_driver/CartesianTrajectoryConstraint")
(setf (get kortex_driver::CartesianTrajectoryConstraint :definition-)
      "
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



(provide :kortex_driver/CartesianTrajectoryConstraint "4758c371e633c2206a71f0c6833f6d5a")


