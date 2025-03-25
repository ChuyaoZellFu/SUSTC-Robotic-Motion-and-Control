;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PreComputedJointTrajectory)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PreComputedJointTrajectory (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PRECOMPUTEDJOINTTRAJECTORY")
  (make-package "KORTEX_DRIVER::PRECOMPUTEDJOINTTRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude PreComputedJointTrajectory.msg.html


(defclass kortex_driver::PreComputedJointTrajectory
  :super ros::object
  :slots (_mode _trajectory_elements ))

(defmethod kortex_driver::PreComputedJointTrajectory
  (:init
   (&key
    ((:mode __mode) 0)
    ((:trajectory_elements __trajectory_elements) ())
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _trajectory_elements __trajectory_elements)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:trajectory_elements
   (&rest __trajectory_elements)
   (if (keywordp (car __trajectory_elements))
       (send* _trajectory_elements __trajectory_elements)
     (progn
       (if __trajectory_elements (setq _trajectory_elements (car __trajectory_elements)))
       _trajectory_elements)))
  (:serialization-length
   ()
   (+
    ;; uint32 _mode
    4
    ;; kortex_driver/PreComputedJointTrajectoryElement[] _trajectory_elements
    (apply #'+ (send-all _trajectory_elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _mode
       (write-long _mode s)
     ;; kortex_driver/PreComputedJointTrajectoryElement[] _trajectory_elements
     (write-long (length _trajectory_elements) s)
     (dolist (elem _trajectory_elements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/PreComputedJointTrajectoryElement[] _trajectory_elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory_elements (let (r) (dotimes (i n) (push (instance kortex_driver::PreComputedJointTrajectoryElement :init) r)) r))
     (dolist (elem- _trajectory_elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::PreComputedJointTrajectory :md5sum-) "0b9aae85f07814f6f429b1dde77280f6")
(setf (get kortex_driver::PreComputedJointTrajectory :datatype-) "kortex_driver/PreComputedJointTrajectory")
(setf (get kortex_driver::PreComputedJointTrajectory :definition-)
      "
uint32 mode
PreComputedJointTrajectoryElement[] trajectory_elements
================================================================================
MSG: kortex_driver/PreComputedJointTrajectoryElement

float32[] joint_angles
float32[] joint_speeds
float32[] joint_accelerations
float32 time_from_start
")



(provide :kortex_driver/PreComputedJointTrajectory "0b9aae85f07814f6f429b1dde77280f6")


