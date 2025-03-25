;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::AxisOffsets)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'AxisOffsets (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::AXISOFFSETS")
  (make-package "KORTEX_DRIVER::AXISOFFSETS"))

(in-package "ROS")
;;//! \htmlinclude AxisOffsets.msg.html


(defclass kortex_driver::AxisOffsets
  :super ros::object
  :slots (_absolute_offset _relative_offset ))

(defmethod kortex_driver::AxisOffsets
  (:init
   (&key
    ((:absolute_offset __absolute_offset) 0.0)
    ((:relative_offset __relative_offset) 0.0)
    )
   (send-super :init)
   (setq _absolute_offset (float __absolute_offset))
   (setq _relative_offset (float __relative_offset))
   self)
  (:absolute_offset
   (&optional __absolute_offset)
   (if __absolute_offset (setq _absolute_offset __absolute_offset)) _absolute_offset)
  (:relative_offset
   (&optional __relative_offset)
   (if __relative_offset (setq _relative_offset __relative_offset)) _relative_offset)
  (:serialization-length
   ()
   (+
    ;; float32 _absolute_offset
    4
    ;; float32 _relative_offset
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _absolute_offset
       (sys::poke _absolute_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _relative_offset
       (sys::poke _relative_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _absolute_offset
     (setq _absolute_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _relative_offset
     (setq _relative_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::AxisOffsets :md5sum-) "2cf10c1327fbdc40ecc0a83625568f5b")
(setf (get kortex_driver::AxisOffsets :datatype-) "kortex_driver/AxisOffsets")
(setf (get kortex_driver::AxisOffsets :definition-)
      "
float32 absolute_offset
float32 relative_offset
")



(provide :kortex_driver/AxisOffsets "2cf10c1327fbdc40ecc0a83625568f5b")


