;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TrajectoryErrorReport)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TrajectoryErrorReport (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRAJECTORYERRORREPORT")
  (make-package "KORTEX_DRIVER::TRAJECTORYERRORREPORT"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryErrorReport.msg.html


(defclass kortex_driver::TrajectoryErrorReport
  :super ros::object
  :slots (_trajectory_error_elements ))

(defmethod kortex_driver::TrajectoryErrorReport
  (:init
   (&key
    ((:trajectory_error_elements __trajectory_error_elements) ())
    )
   (send-super :init)
   (setq _trajectory_error_elements __trajectory_error_elements)
   self)
  (:trajectory_error_elements
   (&rest __trajectory_error_elements)
   (if (keywordp (car __trajectory_error_elements))
       (send* _trajectory_error_elements __trajectory_error_elements)
     (progn
       (if __trajectory_error_elements (setq _trajectory_error_elements (car __trajectory_error_elements)))
       _trajectory_error_elements)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/TrajectoryErrorElement[] _trajectory_error_elements
    (apply #'+ (send-all _trajectory_error_elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TrajectoryErrorElement[] _trajectory_error_elements
     (write-long (length _trajectory_error_elements) s)
     (dolist (elem _trajectory_error_elements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TrajectoryErrorElement[] _trajectory_error_elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory_error_elements (let (r) (dotimes (i n) (push (instance kortex_driver::TrajectoryErrorElement :init) r)) r))
     (dolist (elem- _trajectory_error_elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::TrajectoryErrorReport :md5sum-) "a94c619ffdcf2a4bccd610a99b960b54")
(setf (get kortex_driver::TrajectoryErrorReport :datatype-) "kortex_driver/TrajectoryErrorReport")
(setf (get kortex_driver::TrajectoryErrorReport :definition-)
      "
TrajectoryErrorElement[] trajectory_error_elements
================================================================================
MSG: kortex_driver/TrajectoryErrorElement

uint32 error_type
uint32 error_identifier
float32 error_value
float32 min_value
float32 max_value
uint32 index
string message
uint32 waypoint_index
")



(provide :kortex_driver/TrajectoryErrorReport "a94c619ffdcf2a4bccd610a99b960b54")


