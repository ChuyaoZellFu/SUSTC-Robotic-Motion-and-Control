;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Twist)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Twist (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TWIST")
  (make-package "KORTEX_DRIVER::TWIST"))

(in-package "ROS")
;;//! \htmlinclude Twist.msg.html


(defclass kortex_driver::Twist
  :super ros::object
  :slots (_linear_x _linear_y _linear_z _angular_x _angular_y _angular_z ))

(defmethod kortex_driver::Twist
  (:init
   (&key
    ((:linear_x __linear_x) 0.0)
    ((:linear_y __linear_y) 0.0)
    ((:linear_z __linear_z) 0.0)
    ((:angular_x __angular_x) 0.0)
    ((:angular_y __angular_y) 0.0)
    ((:angular_z __angular_z) 0.0)
    )
   (send-super :init)
   (setq _linear_x (float __linear_x))
   (setq _linear_y (float __linear_y))
   (setq _linear_z (float __linear_z))
   (setq _angular_x (float __angular_x))
   (setq _angular_y (float __angular_y))
   (setq _angular_z (float __angular_z))
   self)
  (:linear_x
   (&optional __linear_x)
   (if __linear_x (setq _linear_x __linear_x)) _linear_x)
  (:linear_y
   (&optional __linear_y)
   (if __linear_y (setq _linear_y __linear_y)) _linear_y)
  (:linear_z
   (&optional __linear_z)
   (if __linear_z (setq _linear_z __linear_z)) _linear_z)
  (:angular_x
   (&optional __angular_x)
   (if __angular_x (setq _angular_x __angular_x)) _angular_x)
  (:angular_y
   (&optional __angular_y)
   (if __angular_y (setq _angular_y __angular_y)) _angular_y)
  (:angular_z
   (&optional __angular_z)
   (if __angular_z (setq _angular_z __angular_z)) _angular_z)
  (:serialization-length
   ()
   (+
    ;; float32 _linear_x
    4
    ;; float32 _linear_y
    4
    ;; float32 _linear_z
    4
    ;; float32 _angular_x
    4
    ;; float32 _angular_y
    4
    ;; float32 _angular_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _linear_x
       (sys::poke _linear_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_y
       (sys::poke _linear_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linear_z
       (sys::poke _linear_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_x
       (sys::poke _angular_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_y
       (sys::poke _angular_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_z
       (sys::poke _angular_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _linear_x
     (setq _linear_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_y
     (setq _linear_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linear_z
     (setq _linear_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_x
     (setq _angular_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_y
     (setq _angular_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_z
     (setq _angular_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Twist :md5sum-) "aadf583bd82c7a5a42adeb6e230fba63")
(setf (get kortex_driver::Twist :datatype-) "kortex_driver/Twist")
(setf (get kortex_driver::Twist :definition-)
      "
float32 linear_x
float32 linear_y
float32 linear_z
float32 angular_x
float32 angular_y
float32 angular_z
")



(provide :kortex_driver/Twist "aadf583bd82c7a5a42adeb6e230fba63")


