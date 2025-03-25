;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ValidateWaypointList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ValidateWaypointList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::VALIDATEWAYPOINTLIST")
  (make-package "KORTEX_DRIVER::VALIDATEWAYPOINTLIST"))
(unless (find-package "KORTEX_DRIVER::VALIDATEWAYPOINTLISTREQUEST")
  (make-package "KORTEX_DRIVER::VALIDATEWAYPOINTLISTREQUEST"))
(unless (find-package "KORTEX_DRIVER::VALIDATEWAYPOINTLISTRESPONSE")
  (make-package "KORTEX_DRIVER::VALIDATEWAYPOINTLISTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ValidateWaypointListRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ValidateWaypointListRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::WaypointList :init))
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
    ;; kortex_driver/WaypointList _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WaypointList _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WaypointList _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ValidateWaypointListResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ValidateWaypointListResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::WaypointValidationReport :init))
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
    ;; kortex_driver/WaypointValidationReport _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WaypointValidationReport _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WaypointValidationReport _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ValidateWaypointList
  :super ros::object
  :slots ())

(setf (get kortex_driver::ValidateWaypointList :md5sum-) "0b24f0cd37f005fabc6c65bffd727f77")
(setf (get kortex_driver::ValidateWaypointList :datatype-) "kortex_driver/ValidateWaypointList")
(setf (get kortex_driver::ValidateWaypointList :request) kortex_driver::ValidateWaypointListRequest)
(setf (get kortex_driver::ValidateWaypointList :response) kortex_driver::ValidateWaypointListResponse)

(defmethod kortex_driver::ValidateWaypointListRequest
  (:response () (instance kortex_driver::ValidateWaypointListResponse :init)))

(setf (get kortex_driver::ValidateWaypointListRequest :md5sum-) "0b24f0cd37f005fabc6c65bffd727f77")
(setf (get kortex_driver::ValidateWaypointListRequest :datatype-) "kortex_driver/ValidateWaypointListRequest")
(setf (get kortex_driver::ValidateWaypointListRequest :definition-)
      "WaypointList input

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
---
WaypointValidationReport output

================================================================================
MSG: kortex_driver/WaypointValidationReport

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
")

(setf (get kortex_driver::ValidateWaypointListResponse :md5sum-) "0b24f0cd37f005fabc6c65bffd727f77")
(setf (get kortex_driver::ValidateWaypointListResponse :datatype-) "kortex_driver/ValidateWaypointListResponse")
(setf (get kortex_driver::ValidateWaypointListResponse :definition-)
      "WaypointList input

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
---
WaypointValidationReport output

================================================================================
MSG: kortex_driver/WaypointValidationReport

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
")



(provide :kortex_driver/ValidateWaypointList "0b24f0cd37f005fabc6c65bffd727f77")


