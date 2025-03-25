;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ExecuteWaypointTrajectory)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ExecuteWaypointTrajectory (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::EXECUTEWAYPOINTTRAJECTORY")
  (make-package "KORTEX_DRIVER::EXECUTEWAYPOINTTRAJECTORY"))
(unless (find-package "KORTEX_DRIVER::EXECUTEWAYPOINTTRAJECTORYREQUEST")
  (make-package "KORTEX_DRIVER::EXECUTEWAYPOINTTRAJECTORYREQUEST"))
(unless (find-package "KORTEX_DRIVER::EXECUTEWAYPOINTTRAJECTORYRESPONSE")
  (make-package "KORTEX_DRIVER::EXECUTEWAYPOINTTRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ExecuteWaypointTrajectoryRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ExecuteWaypointTrajectoryRequest
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

(defclass kortex_driver::ExecuteWaypointTrajectoryResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ExecuteWaypointTrajectoryResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ExecuteWaypointTrajectory
  :super ros::object
  :slots ())

(setf (get kortex_driver::ExecuteWaypointTrajectory :md5sum-) "6b19855f6dc5db92347832c0f2e37c96")
(setf (get kortex_driver::ExecuteWaypointTrajectory :datatype-) "kortex_driver/ExecuteWaypointTrajectory")
(setf (get kortex_driver::ExecuteWaypointTrajectory :request) kortex_driver::ExecuteWaypointTrajectoryRequest)
(setf (get kortex_driver::ExecuteWaypointTrajectory :response) kortex_driver::ExecuteWaypointTrajectoryResponse)

(defmethod kortex_driver::ExecuteWaypointTrajectoryRequest
  (:response () (instance kortex_driver::ExecuteWaypointTrajectoryResponse :init)))

(setf (get kortex_driver::ExecuteWaypointTrajectoryRequest :md5sum-) "6b19855f6dc5db92347832c0f2e37c96")
(setf (get kortex_driver::ExecuteWaypointTrajectoryRequest :datatype-) "kortex_driver/ExecuteWaypointTrajectoryRequest")
(setf (get kortex_driver::ExecuteWaypointTrajectoryRequest :definition-)
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
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ExecuteWaypointTrajectoryResponse :md5sum-) "6b19855f6dc5db92347832c0f2e37c96")
(setf (get kortex_driver::ExecuteWaypointTrajectoryResponse :datatype-) "kortex_driver/ExecuteWaypointTrajectoryResponse")
(setf (get kortex_driver::ExecuteWaypointTrajectoryResponse :definition-)
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
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ExecuteWaypointTrajectory "6b19855f6dc5db92347832c0f2e37c96")


