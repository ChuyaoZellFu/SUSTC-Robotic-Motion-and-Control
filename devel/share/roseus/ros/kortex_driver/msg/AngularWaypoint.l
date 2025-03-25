;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::AngularWaypoint)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'AngularWaypoint (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ANGULARWAYPOINT")
  (make-package "KORTEX_DRIVER::ANGULARWAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude AngularWaypoint.msg.html


(defclass kortex_driver::AngularWaypoint
  :super ros::object
  :slots (_angles _maximum_velocities _duration ))

(defmethod kortex_driver::AngularWaypoint
  (:init
   (&key
    ((:angles __angles) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:maximum_velocities __maximum_velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _angles __angles)
   (setq _maximum_velocities __maximum_velocities)
   (setq _duration (float __duration))
   self)
  (:angles
   (&optional __angles)
   (if __angles (setq _angles __angles)) _angles)
  (:maximum_velocities
   (&optional __maximum_velocities)
   (if __maximum_velocities (setq _maximum_velocities __maximum_velocities)) _maximum_velocities)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32[] _angles
    (* 4    (length _angles)) 4
    ;; float32[] _maximum_velocities
    (* 4    (length _maximum_velocities)) 4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _angles
     (write-long (length _angles) s)
     (dotimes (i (length _angles))
       (sys::poke (elt _angles i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _maximum_velocities
     (write-long (length _maximum_velocities) s)
     (dotimes (i (length _maximum_velocities))
       (sys::poke (elt _maximum_velocities i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angles i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _maximum_velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _maximum_velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _maximum_velocities i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::AngularWaypoint :md5sum-) "c2389f0b849f9b353e58e97764b9bfc7")
(setf (get kortex_driver::AngularWaypoint :datatype-) "kortex_driver/AngularWaypoint")
(setf (get kortex_driver::AngularWaypoint :definition-)
      "
float32[] angles
float32[] maximum_velocities
float32 duration
")



(provide :kortex_driver/AngularWaypoint "c2389f0b849f9b353e58e97764b9bfc7")


