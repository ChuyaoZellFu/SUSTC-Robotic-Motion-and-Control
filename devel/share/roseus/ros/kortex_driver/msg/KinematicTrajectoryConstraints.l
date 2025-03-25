;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::KinematicTrajectoryConstraints)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'KinematicTrajectoryConstraints (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::KINEMATICTRAJECTORYCONSTRAINTS")
  (make-package "KORTEX_DRIVER::KINEMATICTRAJECTORYCONSTRAINTS"))

(in-package "ROS")
;;//! \htmlinclude KinematicTrajectoryConstraints.msg.html


(defclass kortex_driver::KinematicTrajectoryConstraints
  :super ros::object
  :slots (_angular_velocities _linear_velocity _angular_velocity ))

(defmethod kortex_driver::KinematicTrajectoryConstraints
  (:init
   (&key
    ((:angular_velocities __angular_velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:linear_velocity __linear_velocity) 0.0)
    ((:angular_velocity __angular_velocity) 0.0)
    )
   (send-super :init)
   (setq _angular_velocities __angular_velocities)
   (setq _linear_velocity (float __linear_velocity))
   (setq _angular_velocity (float __angular_velocity))
   self)
  (:angular_velocities
   (&optional __angular_velocities)
   (if __angular_velocities (setq _angular_velocities __angular_velocities)) _angular_velocities)
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:angular_velocity
   (&optional __angular_velocity)
   (if __angular_velocity (setq _angular_velocity __angular_velocity)) _angular_velocity)
  (:serialization-length
   ()
   (+
    ;; float32[] _angular_velocities
    (* 4    (length _angular_velocities)) 4
    ;; float32 _linear_velocity
    4
    ;; float32 _angular_velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _angular_velocities
     (write-long (length _angular_velocities) s)
     (dotimes (i (length _angular_velocities))
       (sys::poke (elt _angular_velocities i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _linear_velocity
       (sys::poke _linear_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity
       (sys::poke _angular_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _angular_velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angular_velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angular_velocities i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _linear_velocity
     (setq _linear_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity
     (setq _angular_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::KinematicTrajectoryConstraints :md5sum-) "4f0c906b29207ad54df6adc023e55732")
(setf (get kortex_driver::KinematicTrajectoryConstraints :datatype-) "kortex_driver/KinematicTrajectoryConstraints")
(setf (get kortex_driver::KinematicTrajectoryConstraints :definition-)
      "
float32[] angular_velocities
float32 linear_velocity
float32 angular_velocity
")



(provide :kortex_driver/KinematicTrajectoryConstraints "4f0c906b29207ad54df6adc023e55732")


