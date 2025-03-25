;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointTrajectoryConstraint)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointTrajectoryConstraint (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINT")
  (make-package "KORTEX_DRIVER::JOINTTRAJECTORYCONSTRAINT"))

(in-package "ROS")
;;//! \htmlinclude JointTrajectoryConstraint.msg.html


(defclass kortex_driver::JointTrajectoryConstraint
  :super ros::object
  :slots (_type _value ))

(defmethod kortex_driver::JointTrajectoryConstraint
  (:init
   (&key
    ((:type __type) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _value (float __value))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _type
    4
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _type
       (write-long _type s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::JointTrajectoryConstraint :md5sum-) "db87d33773847285bf5558a40e0eb92c")
(setf (get kortex_driver::JointTrajectoryConstraint :datatype-) "kortex_driver/JointTrajectoryConstraint")
(setf (get kortex_driver::JointTrajectoryConstraint :definition-)
      "
uint32 type
float32 value
")



(provide :kortex_driver/JointTrajectoryConstraint "db87d33773847285bf5558a40e0eb92c")


