;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::KinematicLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'KinematicLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::KINEMATICLIMITS")
  (make-package "KORTEX_DRIVER::KINEMATICLIMITS"))

(in-package "ROS")
;;//! \htmlinclude KinematicLimits.msg.html


(defclass kortex_driver::KinematicLimits
  :super ros::object
  :slots (_control_mode _twist_linear _twist_angular _joint_speed_limits _joint_acceleration_limits ))

(defmethod kortex_driver::KinematicLimits
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    ((:twist_linear __twist_linear) 0.0)
    ((:twist_angular __twist_angular) 0.0)
    ((:joint_speed_limits __joint_speed_limits) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_acceleration_limits __joint_acceleration_limits) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   (setq _twist_linear (float __twist_linear))
   (setq _twist_angular (float __twist_angular))
   (setq _joint_speed_limits __joint_speed_limits)
   (setq _joint_acceleration_limits __joint_acceleration_limits)
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:twist_linear
   (&optional __twist_linear)
   (if __twist_linear (setq _twist_linear __twist_linear)) _twist_linear)
  (:twist_angular
   (&optional __twist_angular)
   (if __twist_angular (setq _twist_angular __twist_angular)) _twist_angular)
  (:joint_speed_limits
   (&optional __joint_speed_limits)
   (if __joint_speed_limits (setq _joint_speed_limits __joint_speed_limits)) _joint_speed_limits)
  (:joint_acceleration_limits
   (&optional __joint_acceleration_limits)
   (if __joint_acceleration_limits (setq _joint_acceleration_limits __joint_acceleration_limits)) _joint_acceleration_limits)
  (:serialization-length
   ()
   (+
    ;; uint32 _control_mode
    4
    ;; float32 _twist_linear
    4
    ;; float32 _twist_angular
    4
    ;; float32[] _joint_speed_limits
    (* 4    (length _joint_speed_limits)) 4
    ;; float32[] _joint_acceleration_limits
    (* 4    (length _joint_acceleration_limits)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _control_mode
       (write-long _control_mode s)
     ;; float32 _twist_linear
       (sys::poke _twist_linear (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _twist_angular
       (sys::poke _twist_angular (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _joint_speed_limits
     (write-long (length _joint_speed_limits) s)
     (dotimes (i (length _joint_speed_limits))
       (sys::poke (elt _joint_speed_limits i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _joint_acceleration_limits
     (write-long (length _joint_acceleration_limits) s)
     (dotimes (i (length _joint_acceleration_limits))
       (sys::poke (elt _joint_acceleration_limits i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _control_mode
     (setq _control_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _twist_linear
     (setq _twist_linear (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _twist_angular
     (setq _twist_angular (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _joint_speed_limits
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_speed_limits (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_speed_limits i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _joint_acceleration_limits
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_acceleration_limits (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_acceleration_limits i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::KinematicLimits :md5sum-) "5496cda94486ec0fe4a0697dda6200aa")
(setf (get kortex_driver::KinematicLimits :datatype-) "kortex_driver/KinematicLimits")
(setf (get kortex_driver::KinematicLimits :definition-)
      "
uint32 control_mode
float32 twist_linear
float32 twist_angular
float32[] joint_speed_limits
float32[] joint_acceleration_limits
")



(provide :kortex_driver/KinematicLimits "5496cda94486ec0fe4a0697dda6200aa")


