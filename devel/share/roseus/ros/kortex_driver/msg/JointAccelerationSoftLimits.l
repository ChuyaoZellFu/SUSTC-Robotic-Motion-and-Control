;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointAccelerationSoftLimits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointAccelerationSoftLimits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTACCELERATIONSOFTLIMITS")
  (make-package "KORTEX_DRIVER::JOINTACCELERATIONSOFTLIMITS"))

(in-package "ROS")
;;//! \htmlinclude JointAccelerationSoftLimits.msg.html


(defclass kortex_driver::JointAccelerationSoftLimits
  :super ros::object
  :slots (_control_mode _joint_acceleration_soft_limits ))

(defmethod kortex_driver::JointAccelerationSoftLimits
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    ((:joint_acceleration_soft_limits __joint_acceleration_soft_limits) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   (setq _joint_acceleration_soft_limits __joint_acceleration_soft_limits)
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:joint_acceleration_soft_limits
   (&optional __joint_acceleration_soft_limits)
   (if __joint_acceleration_soft_limits (setq _joint_acceleration_soft_limits __joint_acceleration_soft_limits)) _joint_acceleration_soft_limits)
  (:serialization-length
   ()
   (+
    ;; uint32 _control_mode
    4
    ;; float32[] _joint_acceleration_soft_limits
    (* 4    (length _joint_acceleration_soft_limits)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _control_mode
       (write-long _control_mode s)
     ;; float32[] _joint_acceleration_soft_limits
     (write-long (length _joint_acceleration_soft_limits) s)
     (dotimes (i (length _joint_acceleration_soft_limits))
       (sys::poke (elt _joint_acceleration_soft_limits i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _control_mode
     (setq _control_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _joint_acceleration_soft_limits
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_acceleration_soft_limits (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_acceleration_soft_limits i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::JointAccelerationSoftLimits :md5sum-) "69f7da5340a05ca3142f1af9be835bb9")
(setf (get kortex_driver::JointAccelerationSoftLimits :datatype-) "kortex_driver/JointAccelerationSoftLimits")
(setf (get kortex_driver::JointAccelerationSoftLimits :definition-)
      "
uint32 control_mode
float32[] joint_acceleration_soft_limits
")



(provide :kortex_driver/JointAccelerationSoftLimits "69f7da5340a05ca3142f1af9be835bb9")


