;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Waypoint)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Waypoint (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WAYPOINT")
  (make-package "KORTEX_DRIVER::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html


(defclass kortex_driver::Waypoint
  :super ros::object
  :slots (_name _oneof_type_of_waypoint ))

(defmethod kortex_driver::Waypoint
  (:init
   (&key
    ((:name __name) "")
    ((:oneof_type_of_waypoint __oneof_type_of_waypoint) (instance kortex_driver::Waypoint_type_of_waypoint :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _oneof_type_of_waypoint __oneof_type_of_waypoint)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:oneof_type_of_waypoint
   (&rest __oneof_type_of_waypoint)
   (if (keywordp (car __oneof_type_of_waypoint))
       (send* _oneof_type_of_waypoint __oneof_type_of_waypoint)
     (progn
       (if __oneof_type_of_waypoint (setq _oneof_type_of_waypoint (car __oneof_type_of_waypoint)))
       _oneof_type_of_waypoint)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; kortex_driver/Waypoint_type_of_waypoint _oneof_type_of_waypoint
    (send _oneof_type_of_waypoint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; kortex_driver/Waypoint_type_of_waypoint _oneof_type_of_waypoint
       (send _oneof_type_of_waypoint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kortex_driver/Waypoint_type_of_waypoint _oneof_type_of_waypoint
     (send _oneof_type_of_waypoint :deserialize buf ptr-) (incf ptr- (send _oneof_type_of_waypoint :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::Waypoint :md5sum-) "936edf0520133d6221befe691467b5ce")
(setf (get kortex_driver::Waypoint :datatype-) "kortex_driver/Waypoint")
(setf (get kortex_driver::Waypoint :definition-)
      "
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



(provide :kortex_driver/Waypoint "936edf0520133d6221befe691467b5ce")


