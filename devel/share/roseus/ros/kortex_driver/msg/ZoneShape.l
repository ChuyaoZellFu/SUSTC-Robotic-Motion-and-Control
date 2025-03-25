;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ZoneShape)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ZoneShape (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ZONESHAPE")
  (make-package "KORTEX_DRIVER::ZONESHAPE"))

(in-package "ROS")
;;//! \htmlinclude ZoneShape.msg.html


(defclass kortex_driver::ZoneShape
  :super ros::object
  :slots (_shape_type _origin _orientation _dimensions _envelope_thickness ))

(defmethod kortex_driver::ZoneShape
  (:init
   (&key
    ((:shape_type __shape_type) 0)
    ((:origin __origin) (instance kortex_driver::Point :init))
    ((:orientation __orientation) (instance kortex_driver::Base_RotationMatrix :init))
    ((:dimensions __dimensions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:envelope_thickness __envelope_thickness) 0.0)
    )
   (send-super :init)
   (setq _shape_type (round __shape_type))
   (setq _origin __origin)
   (setq _orientation __orientation)
   (setq _dimensions __dimensions)
   (setq _envelope_thickness (float __envelope_thickness))
   self)
  (:shape_type
   (&optional __shape_type)
   (if __shape_type (setq _shape_type __shape_type)) _shape_type)
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:dimensions
   (&optional __dimensions)
   (if __dimensions (setq _dimensions __dimensions)) _dimensions)
  (:envelope_thickness
   (&optional __envelope_thickness)
   (if __envelope_thickness (setq _envelope_thickness __envelope_thickness)) _envelope_thickness)
  (:serialization-length
   ()
   (+
    ;; uint32 _shape_type
    4
    ;; kortex_driver/Point _origin
    (send _origin :serialization-length)
    ;; kortex_driver/Base_RotationMatrix _orientation
    (send _orientation :serialization-length)
    ;; float32[] _dimensions
    (* 4    (length _dimensions)) 4
    ;; float32 _envelope_thickness
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _shape_type
       (write-long _shape_type s)
     ;; kortex_driver/Point _origin
       (send _origin :serialize s)
     ;; kortex_driver/Base_RotationMatrix _orientation
       (send _orientation :serialize s)
     ;; float32[] _dimensions
     (write-long (length _dimensions) s)
     (dotimes (i (length _dimensions))
       (sys::poke (elt _dimensions i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _envelope_thickness
       (sys::poke _envelope_thickness (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _shape_type
     (setq _shape_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Point _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; kortex_driver/Base_RotationMatrix _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;; float32[] _dimensions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dimensions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dimensions i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _envelope_thickness
     (setq _envelope_thickness (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ZoneShape :md5sum-) "8de31189535cb06cb8caf5fc49d6c1b4")
(setf (get kortex_driver::ZoneShape :datatype-) "kortex_driver/ZoneShape")
(setf (get kortex_driver::ZoneShape :definition-)
      "
uint32 shape_type
Point origin
Base_RotationMatrix orientation
float32[] dimensions
float32 envelope_thickness
================================================================================
MSG: kortex_driver/Point

float32 x
float32 y
float32 z
================================================================================
MSG: kortex_driver/Base_RotationMatrix

Base_RotationMatrixRow row1
Base_RotationMatrixRow row2
Base_RotationMatrixRow row3
================================================================================
MSG: kortex_driver/Base_RotationMatrixRow

float32 column1
float32 column2
float32 column3
")



(provide :kortex_driver/ZoneShape "8de31189535cb06cb8caf5fc49d6c1b4")


