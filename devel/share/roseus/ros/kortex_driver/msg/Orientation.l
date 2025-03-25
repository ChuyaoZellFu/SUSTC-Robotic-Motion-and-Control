;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Orientation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Orientation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ORIENTATION")
  (make-package "KORTEX_DRIVER::ORIENTATION"))

(in-package "ROS")
;;//! \htmlinclude Orientation.msg.html


(defclass kortex_driver::Orientation
  :super ros::object
  :slots (_theta_x _theta_y _theta_z ))

(defmethod kortex_driver::Orientation
  (:init
   (&key
    ((:theta_x __theta_x) 0.0)
    ((:theta_y __theta_y) 0.0)
    ((:theta_z __theta_z) 0.0)
    )
   (send-super :init)
   (setq _theta_x (float __theta_x))
   (setq _theta_y (float __theta_y))
   (setq _theta_z (float __theta_z))
   self)
  (:theta_x
   (&optional __theta_x)
   (if __theta_x (setq _theta_x __theta_x)) _theta_x)
  (:theta_y
   (&optional __theta_y)
   (if __theta_y (setq _theta_y __theta_y)) _theta_y)
  (:theta_z
   (&optional __theta_z)
   (if __theta_z (setq _theta_z __theta_z)) _theta_z)
  (:serialization-length
   ()
   (+
    ;; float32 _theta_x
    4
    ;; float32 _theta_y
    4
    ;; float32 _theta_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _theta_x
       (sys::poke _theta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_y
       (sys::poke _theta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_z
       (sys::poke _theta_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _theta_x
     (setq _theta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_y
     (setq _theta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_z
     (setq _theta_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Orientation :md5sum-) "2fb1d2b19db735698f3a1ad8c8e9d45b")
(setf (get kortex_driver::Orientation :datatype-) "kortex_driver/Orientation")
(setf (get kortex_driver::Orientation :definition-)
      "
float32 theta_x
float32 theta_y
float32 theta_z
")



(provide :kortex_driver/Orientation "2fb1d2b19db735698f3a1ad8c8e9d45b")


