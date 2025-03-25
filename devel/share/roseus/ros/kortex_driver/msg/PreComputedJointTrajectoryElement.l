;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PreComputedJointTrajectoryElement)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PreComputedJointTrajectoryElement (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PRECOMPUTEDJOINTTRAJECTORYELEMENT")
  (make-package "KORTEX_DRIVER::PRECOMPUTEDJOINTTRAJECTORYELEMENT"))

(in-package "ROS")
;;//! \htmlinclude PreComputedJointTrajectoryElement.msg.html


(defclass kortex_driver::PreComputedJointTrajectoryElement
  :super ros::object
  :slots (_joint_angles _joint_speeds _joint_accelerations _time_from_start ))

(defmethod kortex_driver::PreComputedJointTrajectoryElement
  (:init
   (&key
    ((:joint_angles __joint_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_speeds __joint_speeds) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_accelerations __joint_accelerations) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:time_from_start __time_from_start) 0.0)
    )
   (send-super :init)
   (setq _joint_angles __joint_angles)
   (setq _joint_speeds __joint_speeds)
   (setq _joint_accelerations __joint_accelerations)
   (setq _time_from_start (float __time_from_start))
   self)
  (:joint_angles
   (&optional __joint_angles)
   (if __joint_angles (setq _joint_angles __joint_angles)) _joint_angles)
  (:joint_speeds
   (&optional __joint_speeds)
   (if __joint_speeds (setq _joint_speeds __joint_speeds)) _joint_speeds)
  (:joint_accelerations
   (&optional __joint_accelerations)
   (if __joint_accelerations (setq _joint_accelerations __joint_accelerations)) _joint_accelerations)
  (:time_from_start
   (&optional __time_from_start)
   (if __time_from_start (setq _time_from_start __time_from_start)) _time_from_start)
  (:serialization-length
   ()
   (+
    ;; float32[] _joint_angles
    (* 4    (length _joint_angles)) 4
    ;; float32[] _joint_speeds
    (* 4    (length _joint_speeds)) 4
    ;; float32[] _joint_accelerations
    (* 4    (length _joint_accelerations)) 4
    ;; float32 _time_from_start
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _joint_angles
     (write-long (length _joint_angles) s)
     (dotimes (i (length _joint_angles))
       (sys::poke (elt _joint_angles i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _joint_speeds
     (write-long (length _joint_speeds) s)
     (dotimes (i (length _joint_speeds))
       (sys::poke (elt _joint_speeds i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _joint_accelerations
     (write-long (length _joint_accelerations) s)
     (dotimes (i (length _joint_accelerations))
       (sys::poke (elt _joint_accelerations i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _time_from_start
       (sys::poke _time_from_start (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _joint_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_angles i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _joint_speeds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_speeds (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_speeds i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _joint_accelerations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_accelerations (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_accelerations i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _time_from_start
     (setq _time_from_start (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::PreComputedJointTrajectoryElement :md5sum-) "da991996b59092cd15d7abc565fb075e")
(setf (get kortex_driver::PreComputedJointTrajectoryElement :datatype-) "kortex_driver/PreComputedJointTrajectoryElement")
(setf (get kortex_driver::PreComputedJointTrajectoryElement :definition-)
      "
float32[] joint_angles
float32[] joint_speeds
float32[] joint_accelerations
float32 time_from_start
")



(provide :kortex_driver/PreComputedJointTrajectoryElement "da991996b59092cd15d7abc565fb075e")


