;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DistortionCoefficients)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DistortionCoefficients (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DISTORTIONCOEFFICIENTS")
  (make-package "KORTEX_DRIVER::DISTORTIONCOEFFICIENTS"))

(in-package "ROS")
;;//! \htmlinclude DistortionCoefficients.msg.html


(defclass kortex_driver::DistortionCoefficients
  :super ros::object
  :slots (_k1 _k2 _k3 _p1 _p2 ))

(defmethod kortex_driver::DistortionCoefficients
  (:init
   (&key
    ((:k1 __k1) 0.0)
    ((:k2 __k2) 0.0)
    ((:k3 __k3) 0.0)
    ((:p1 __p1) 0.0)
    ((:p2 __p2) 0.0)
    )
   (send-super :init)
   (setq _k1 (float __k1))
   (setq _k2 (float __k2))
   (setq _k3 (float __k3))
   (setq _p1 (float __p1))
   (setq _p2 (float __p2))
   self)
  (:k1
   (&optional __k1)
   (if __k1 (setq _k1 __k1)) _k1)
  (:k2
   (&optional __k2)
   (if __k2 (setq _k2 __k2)) _k2)
  (:k3
   (&optional __k3)
   (if __k3 (setq _k3 __k3)) _k3)
  (:p1
   (&optional __p1)
   (if __p1 (setq _p1 __p1)) _p1)
  (:p2
   (&optional __p2)
   (if __p2 (setq _p2 __p2)) _p2)
  (:serialization-length
   ()
   (+
    ;; float32 _k1
    4
    ;; float32 _k2
    4
    ;; float32 _k3
    4
    ;; float32 _p1
    4
    ;; float32 _p2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _k1
       (sys::poke _k1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _k2
       (sys::poke _k2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _k3
       (sys::poke _k3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p1
       (sys::poke _p1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p2
       (sys::poke _p2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _k1
     (setq _k1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _k2
     (setq _k2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _k3
     (setq _k3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p1
     (setq _p1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p2
     (setq _p2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::DistortionCoefficients :md5sum-) "3d597195629c570d527b87022489fa04")
(setf (get kortex_driver::DistortionCoefficients :datatype-) "kortex_driver/DistortionCoefficients")
(setf (get kortex_driver::DistortionCoefficients :definition-)
      "
float32 k1
float32 k2
float32 k3
float32 p1
float32 p2
")



(provide :kortex_driver/DistortionCoefficients "3d597195629c570d527b87022489fa04")


