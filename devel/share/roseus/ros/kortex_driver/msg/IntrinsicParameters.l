;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::IntrinsicParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'IntrinsicParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTRINSICPARAMETERS")
  (make-package "KORTEX_DRIVER::INTRINSICPARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude IntrinsicParameters.msg.html


(defclass kortex_driver::IntrinsicParameters
  :super ros::object
  :slots (_sensor _resolution _principal_point_x _principal_point_y _focal_length_x _focal_length_y _distortion_coeffs ))

(defmethod kortex_driver::IntrinsicParameters
  (:init
   (&key
    ((:sensor __sensor) 0)
    ((:resolution __resolution) 0)
    ((:principal_point_x __principal_point_x) 0.0)
    ((:principal_point_y __principal_point_y) 0.0)
    ((:focal_length_x __focal_length_x) 0.0)
    ((:focal_length_y __focal_length_y) 0.0)
    ((:distortion_coeffs __distortion_coeffs) (instance kortex_driver::DistortionCoefficients :init))
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   (setq _resolution (round __resolution))
   (setq _principal_point_x (float __principal_point_x))
   (setq _principal_point_y (float __principal_point_y))
   (setq _focal_length_x (float __focal_length_x))
   (setq _focal_length_y (float __focal_length_y))
   (setq _distortion_coeffs __distortion_coeffs)
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:principal_point_x
   (&optional __principal_point_x)
   (if __principal_point_x (setq _principal_point_x __principal_point_x)) _principal_point_x)
  (:principal_point_y
   (&optional __principal_point_y)
   (if __principal_point_y (setq _principal_point_y __principal_point_y)) _principal_point_y)
  (:focal_length_x
   (&optional __focal_length_x)
   (if __focal_length_x (setq _focal_length_x __focal_length_x)) _focal_length_x)
  (:focal_length_y
   (&optional __focal_length_y)
   (if __focal_length_y (setq _focal_length_y __focal_length_y)) _focal_length_y)
  (:distortion_coeffs
   (&rest __distortion_coeffs)
   (if (keywordp (car __distortion_coeffs))
       (send* _distortion_coeffs __distortion_coeffs)
     (progn
       (if __distortion_coeffs (setq _distortion_coeffs (car __distortion_coeffs)))
       _distortion_coeffs)))
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ;; uint32 _resolution
    4
    ;; float32 _principal_point_x
    4
    ;; float32 _principal_point_y
    4
    ;; float32 _focal_length_x
    4
    ;; float32 _focal_length_y
    4
    ;; kortex_driver/DistortionCoefficients _distortion_coeffs
    (send _distortion_coeffs :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sensor
       (write-long _sensor s)
     ;; uint32 _resolution
       (write-long _resolution s)
     ;; float32 _principal_point_x
       (sys::poke _principal_point_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _principal_point_y
       (sys::poke _principal_point_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _focal_length_x
       (sys::poke _focal_length_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _focal_length_y
       (sys::poke _focal_length_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; kortex_driver/DistortionCoefficients _distortion_coeffs
       (send _distortion_coeffs :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _resolution
     (setq _resolution (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _principal_point_x
     (setq _principal_point_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _principal_point_y
     (setq _principal_point_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _focal_length_x
     (setq _focal_length_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _focal_length_y
     (setq _focal_length_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; kortex_driver/DistortionCoefficients _distortion_coeffs
     (send _distortion_coeffs :deserialize buf ptr-) (incf ptr- (send _distortion_coeffs :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::IntrinsicParameters :md5sum-) "23b95f71707877a1e06e2ac8ba309bb9")
(setf (get kortex_driver::IntrinsicParameters :datatype-) "kortex_driver/IntrinsicParameters")
(setf (get kortex_driver::IntrinsicParameters :definition-)
      "
uint32 sensor
uint32 resolution
float32 principal_point_x
float32 principal_point_y
float32 focal_length_x
float32 focal_length_y
DistortionCoefficients distortion_coeffs
================================================================================
MSG: kortex_driver/DistortionCoefficients

float32 k1
float32 k2
float32 k3
float32 p1
float32 p2
")



(provide :kortex_driver/IntrinsicParameters "23b95f71707877a1e06e2ac8ba309bb9")


