;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WaypointValidationReport)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WaypointValidationReport (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WAYPOINTVALIDATIONREPORT")
  (make-package "KORTEX_DRIVER::WAYPOINTVALIDATIONREPORT"))

(in-package "ROS")
;;//! \htmlinclude WaypointValidationReport.msg.html


(defclass kortex_driver::WaypointValidationReport
  :super ros::object
  :slots (_trajectory_error_report _optimal_waypoint_list ))

(defmethod kortex_driver::WaypointValidationReport
  (:init
   (&key
    ((:trajectory_error_report __trajectory_error_report) (instance kortex_driver::TrajectoryErrorReport :init))
    ((:optimal_waypoint_list __optimal_waypoint_list) (instance kortex_driver::WaypointList :init))
    )
   (send-super :init)
   (setq _trajectory_error_report __trajectory_error_report)
   (setq _optimal_waypoint_list __optimal_waypoint_list)
   self)
  (:trajectory_error_report
   (&rest __trajectory_error_report)
   (if (keywordp (car __trajectory_error_report))
       (send* _trajectory_error_report __trajectory_error_report)
     (progn
       (if __trajectory_error_report (setq _trajectory_error_report (car __trajectory_error_report)))
       _trajectory_error_report)))
  (:optimal_waypoint_list
   (&rest __optimal_waypoint_list)
   (if (keywordp (car __optimal_waypoint_list))
       (send* _optimal_waypoint_list __optimal_waypoint_list)
     (progn
       (if __optimal_waypoint_list (setq _optimal_waypoint_list (car __optimal_waypoint_list)))
       _optimal_waypoint_list)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/TrajectoryErrorReport _trajectory_error_report
    (send _trajectory_error_report :serialization-length)
    ;; kortex_driver/WaypointList _optimal_waypoint_list
    (send _optimal_waypoint_list :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TrajectoryErrorReport _trajectory_error_report
       (send _trajectory_error_report :serialize s)
     ;; kortex_driver/WaypointList _optimal_waypoint_list
       (send _optimal_waypoint_list :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TrajectoryErrorReport _trajectory_error_report
     (send _trajectory_error_report :deserialize buf ptr-) (incf ptr- (send _trajectory_error_report :serialization-length))
   ;; kortex_driver/WaypointList _optimal_waypoint_list
     (send _optimal_waypoint_list :deserialize buf ptr-) (incf ptr- (send _optimal_waypoint_list :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::WaypointValidationReport :md5sum-) "a7dd565ec93d4c9c5950e62db23f8114")
(setf (get kortex_driver::WaypointValidationReport :datatype-) "kortex_driver/WaypointValidationReport")
(setf (get kortex_driver::WaypointValidationReport :definition-)
      "
TrajectoryErrorReport trajectory_error_report
WaypointList optimal_waypoint_list
================================================================================
MSG: kortex_driver/TrajectoryErrorReport

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
================================================================================
MSG: kortex_driver/WaypointList

Waypoint[] waypoints
float32 duration
bool use_optimal_blending
================================================================================
MSG: kortex_driver/Waypoint

string name
Waypoint_type_of_waypoint oneof_type_of_waypoint
================================================================================
MSG: kortex_driver/Waypoint_type_of_waypoint

AngularWaypoint[] angular_waypoint
CartesianWaypoint[] cartesian_waypoint
================================================================================
MSG: kortex_driver/AngularWaypoint

float32[] angles
float32[] maximum_velocities
float32 duration
================================================================================
MSG: kortex_driver/CartesianWaypoint

Pose pose
uint32 reference_frame
float32 maximum_linear_velocity
float32 maximum_angular_velocity
float32 blending_radius
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
")



(provide :kortex_driver/WaypointValidationReport "a7dd565ec93d4c9c5950e62db23f8114")


