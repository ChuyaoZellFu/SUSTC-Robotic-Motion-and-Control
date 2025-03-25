;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::AxisPosition)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'AxisPosition (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::AXISPOSITION")
  (make-package "KORTEX_DRIVER::AXISPOSITION"))

(in-package "ROS")
;;//! \htmlinclude AxisPosition.msg.html


(defclass kortex_driver::AxisPosition
  :super ros::object
  :slots (_position ))

(defmethod kortex_driver::AxisPosition
  (:init
   (&key
    ((:position __position) 0.0)
    )
   (send-super :init)
   (setq _position (float __position))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; float32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _position
       (sys::poke _position (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _position
     (setq _position (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::AxisPosition :md5sum-) "e4e11b3af29ed247b2bd150d3f4540f9")
(setf (get kortex_driver::AxisPosition :datatype-) "kortex_driver/AxisPosition")
(setf (get kortex_driver::AxisPosition :definition-)
      "
float32 position
")



(provide :kortex_driver/AxisPosition "e4e11b3af29ed247b2bd150d3f4540f9")


