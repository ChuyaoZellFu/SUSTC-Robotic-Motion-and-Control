;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlConfig_JointSpeeds)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlConfig_JointSpeeds (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIG_JOINTSPEEDS")
  (make-package "KORTEX_DRIVER::CONTROLCONFIG_JOINTSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude ControlConfig_JointSpeeds.msg.html


(defclass kortex_driver::ControlConfig_JointSpeeds
  :super ros::object
  :slots (_joint_speed ))

(defmethod kortex_driver::ControlConfig_JointSpeeds
  (:init
   (&key
    ((:joint_speed __joint_speed) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_speed __joint_speed)
   self)
  (:joint_speed
   (&optional __joint_speed)
   (if __joint_speed (setq _joint_speed __joint_speed)) _joint_speed)
  (:serialization-length
   ()
   (+
    ;; float32[] _joint_speed
    (* 4    (length _joint_speed)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _joint_speed
     (write-long (length _joint_speed) s)
     (dotimes (i (length _joint_speed))
       (sys::poke (elt _joint_speed i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _joint_speed
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_speed (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_speed i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ControlConfig_JointSpeeds :md5sum-) "33fe593b138a7242a779a630fd20122e")
(setf (get kortex_driver::ControlConfig_JointSpeeds :datatype-) "kortex_driver/ControlConfig_JointSpeeds")
(setf (get kortex_driver::ControlConfig_JointSpeeds :definition-)
      "
float32[] joint_speed
")



(provide :kortex_driver/ControlConfig_JointSpeeds "33fe593b138a7242a779a630fd20122e")


