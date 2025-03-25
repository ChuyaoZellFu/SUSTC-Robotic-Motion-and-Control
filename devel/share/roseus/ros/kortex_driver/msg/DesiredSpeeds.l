;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DesiredSpeeds)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DesiredSpeeds (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DESIREDSPEEDS")
  (make-package "KORTEX_DRIVER::DESIREDSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude DesiredSpeeds.msg.html


(defclass kortex_driver::DesiredSpeeds
  :super ros::object
  :slots (_linear _angular _joint_speed ))

(defmethod kortex_driver::DesiredSpeeds
  (:init
   (&key
    ((:linear __linear) 0.0)
    ((:angular __angular) 0.0)
    ((:joint_speed __joint_speed) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _linear (float __linear))
   (setq _angular (float __angular))
   (setq _joint_speed __joint_speed)
   self)
  (:linear
   (&optional __linear)
   (if __linear (setq _linear __linear)) _linear)
  (:angular
   (&optional __angular)
   (if __angular (setq _angular __angular)) _angular)
  (:joint_speed
   (&optional __joint_speed)
   (if __joint_speed (setq _joint_speed __joint_speed)) _joint_speed)
  (:serialization-length
   ()
   (+
    ;; float32 _linear
    4
    ;; float32 _angular
    4
    ;; float32[] _joint_speed
    (* 4    (length _joint_speed)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _linear
       (sys::poke _linear (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular
       (sys::poke _angular (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _joint_speed
     (write-long (length _joint_speed) s)
     (dotimes (i (length _joint_speed))
       (sys::poke (elt _joint_speed i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _linear
     (setq _linear (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular
     (setq _angular (sys::peek buf ptr- :float)) (incf ptr- 4)
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

(setf (get kortex_driver::DesiredSpeeds :md5sum-) "9f61e0fe165a7f46b2508fd0832ff820")
(setf (get kortex_driver::DesiredSpeeds :datatype-) "kortex_driver/DesiredSpeeds")
(setf (get kortex_driver::DesiredSpeeds :definition-)
      "
float32 linear
float32 angular
float32[] joint_speed
")



(provide :kortex_driver/DesiredSpeeds "9f61e0fe165a7f46b2508fd0832ff820")


