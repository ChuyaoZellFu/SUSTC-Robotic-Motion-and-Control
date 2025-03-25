;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WaypointList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WaypointList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WAYPOINTLIST")
  (make-package "KORTEX_DRIVER::WAYPOINTLIST"))

(in-package "ROS")
;;//! \htmlinclude WaypointList.msg.html


(defclass kortex_driver::WaypointList
  :super ros::object
  :slots (_waypoints _duration _use_optimal_blending ))

(defmethod kortex_driver::WaypointList
  (:init
   (&key
    ((:waypoints __waypoints) ())
    ((:duration __duration) 0.0)
    ((:use_optimal_blending __use_optimal_blending) nil)
    )
   (send-super :init)
   (setq _waypoints __waypoints)
   (setq _duration (float __duration))
   (setq _use_optimal_blending __use_optimal_blending)
   self)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:use_optimal_blending
   (&optional (__use_optimal_blending :null))
   (if (not (eq __use_optimal_blending :null)) (setq _use_optimal_blending __use_optimal_blending)) _use_optimal_blending)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ;; float32 _duration
    4
    ;; bool _use_optimal_blending
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _use_optimal_blending
       (if _use_optimal_blending (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Waypoint[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance kortex_driver::Waypoint :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _use_optimal_blending
     (setq _use_optimal_blending (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::WaypointList :md5sum-) "7b5e341798f530d68dce10b5bad29aad")
(setf (get kortex_driver::WaypointList :datatype-) "kortex_driver/WaypointList")
(setf (get kortex_driver::WaypointList :definition-)
      "
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



(provide :kortex_driver/WaypointList "7b5e341798f530d68dce10b5bad29aad")


