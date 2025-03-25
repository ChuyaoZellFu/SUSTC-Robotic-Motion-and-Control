;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Waypoint_type_of_waypoint)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Waypoint_type_of_waypoint (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WAYPOINT_TYPE_OF_WAYPOINT")
  (make-package "KORTEX_DRIVER::WAYPOINT_TYPE_OF_WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint_type_of_waypoint.msg.html


(defclass kortex_driver::Waypoint_type_of_waypoint
  :super ros::object
  :slots (_angular_waypoint _cartesian_waypoint ))

(defmethod kortex_driver::Waypoint_type_of_waypoint
  (:init
   (&key
    ((:angular_waypoint __angular_waypoint) ())
    ((:cartesian_waypoint __cartesian_waypoint) ())
    )
   (send-super :init)
   (setq _angular_waypoint __angular_waypoint)
   (setq _cartesian_waypoint __cartesian_waypoint)
   self)
  (:angular_waypoint
   (&rest __angular_waypoint)
   (if (keywordp (car __angular_waypoint))
       (send* _angular_waypoint __angular_waypoint)
     (progn
       (if __angular_waypoint (setq _angular_waypoint (car __angular_waypoint)))
       _angular_waypoint)))
  (:cartesian_waypoint
   (&rest __cartesian_waypoint)
   (if (keywordp (car __cartesian_waypoint))
       (send* _cartesian_waypoint __cartesian_waypoint)
     (progn
       (if __cartesian_waypoint (setq _cartesian_waypoint (car __cartesian_waypoint)))
       _cartesian_waypoint)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/AngularWaypoint[] _angular_waypoint
    (apply #'+ (send-all _angular_waypoint :serialization-length)) 4
    ;; kortex_driver/CartesianWaypoint[] _cartesian_waypoint
    (apply #'+ (send-all _cartesian_waypoint :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/AngularWaypoint[] _angular_waypoint
     (write-long (length _angular_waypoint) s)
     (dolist (elem _angular_waypoint)
       (send elem :serialize s)
       )
     ;; kortex_driver/CartesianWaypoint[] _cartesian_waypoint
     (write-long (length _cartesian_waypoint) s)
     (dolist (elem _cartesian_waypoint)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/AngularWaypoint[] _angular_waypoint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angular_waypoint (let (r) (dotimes (i n) (push (instance kortex_driver::AngularWaypoint :init) r)) r))
     (dolist (elem- _angular_waypoint)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/CartesianWaypoint[] _cartesian_waypoint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cartesian_waypoint (let (r) (dotimes (i n) (push (instance kortex_driver::CartesianWaypoint :init) r)) r))
     (dolist (elem- _cartesian_waypoint)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::Waypoint_type_of_waypoint :md5sum-) "90682b2ce9c17ef82e25a79e7e035287")
(setf (get kortex_driver::Waypoint_type_of_waypoint :datatype-) "kortex_driver/Waypoint_type_of_waypoint")
(setf (get kortex_driver::Waypoint_type_of_waypoint :definition-)
      "
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



(provide :kortex_driver/Waypoint_type_of_waypoint "90682b2ce9c17ef82e25a79e7e035287")


