;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointAngle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointAngle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTANGLE")
  (make-package "KORTEX_DRIVER::JOINTANGLE"))

(in-package "ROS")
;;//! \htmlinclude JointAngle.msg.html


(defclass kortex_driver::JointAngle
  :super ros::object
  :slots (_joint_identifier _value ))

(defmethod kortex_driver::JointAngle
  (:init
   (&key
    ((:joint_identifier __joint_identifier) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _joint_identifier (round __joint_identifier))
   (setq _value (float __value))
   self)
  (:joint_identifier
   (&optional __joint_identifier)
   (if __joint_identifier (setq _joint_identifier __joint_identifier)) _joint_identifier)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _joint_identifier
    4
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _joint_identifier
       (write-long _joint_identifier s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _joint_identifier
     (setq _joint_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::JointAngle :md5sum-) "a4845050b04cbd4bf489d0d314ebb021")
(setf (get kortex_driver::JointAngle :datatype-) "kortex_driver/JointAngle")
(setf (get kortex_driver::JointAngle :definition-)
      "
uint32 joint_identifier
float32 value
")



(provide :kortex_driver/JointAngle "a4845050b04cbd4bf489d0d314ebb021")


