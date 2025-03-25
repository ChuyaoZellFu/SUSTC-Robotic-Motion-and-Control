;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianWaypoint)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianWaypoint (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANWAYPOINT")
  (make-package "KORTEX_DRIVER::CARTESIANWAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude CartesianWaypoint.msg.html


(defclass kortex_driver::CartesianWaypoint
  :super ros::object
  :slots (_pose _reference_frame _maximum_linear_velocity _maximum_angular_velocity _blending_radius ))

(defmethod kortex_driver::CartesianWaypoint
  (:init
   (&key
    ((:pose __pose) (instance kortex_driver::Pose :init))
    ((:reference_frame __reference_frame) 0)
    ((:maximum_linear_velocity __maximum_linear_velocity) 0.0)
    ((:maximum_angular_velocity __maximum_angular_velocity) 0.0)
    ((:blending_radius __blending_radius) 0.0)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _reference_frame (round __reference_frame))
   (setq _maximum_linear_velocity (float __maximum_linear_velocity))
   (setq _maximum_angular_velocity (float __maximum_angular_velocity))
   (setq _blending_radius (float __blending_radius))
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:maximum_linear_velocity
   (&optional __maximum_linear_velocity)
   (if __maximum_linear_velocity (setq _maximum_linear_velocity __maximum_linear_velocity)) _maximum_linear_velocity)
  (:maximum_angular_velocity
   (&optional __maximum_angular_velocity)
   (if __maximum_angular_velocity (setq _maximum_angular_velocity __maximum_angular_velocity)) _maximum_angular_velocity)
  (:blending_radius
   (&optional __blending_radius)
   (if __blending_radius (setq _blending_radius __blending_radius)) _blending_radius)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Pose _pose
    (send _pose :serialization-length)
    ;; uint32 _reference_frame
    4
    ;; float32 _maximum_linear_velocity
    4
    ;; float32 _maximum_angular_velocity
    4
    ;; float32 _blending_radius
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Pose _pose
       (send _pose :serialize s)
     ;; uint32 _reference_frame
       (write-long _reference_frame s)
     ;; float32 _maximum_linear_velocity
       (sys::poke _maximum_linear_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _maximum_angular_velocity
       (sys::poke _maximum_angular_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _blending_radius
       (sys::poke _blending_radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; uint32 _reference_frame
     (setq _reference_frame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _maximum_linear_velocity
     (setq _maximum_linear_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _maximum_angular_velocity
     (setq _maximum_angular_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _blending_radius
     (setq _blending_radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CartesianWaypoint :md5sum-) "057d3bd32497f1e96612efc0fb6ef690")
(setf (get kortex_driver::CartesianWaypoint :datatype-) "kortex_driver/CartesianWaypoint")
(setf (get kortex_driver::CartesianWaypoint :definition-)
      "
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



(provide :kortex_driver/CartesianWaypoint "057d3bd32497f1e96612efc0fb6ef690")


