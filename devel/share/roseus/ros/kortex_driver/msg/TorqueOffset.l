;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TorqueOffset)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TorqueOffset (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TORQUEOFFSET")
  (make-package "KORTEX_DRIVER::TORQUEOFFSET"))

(in-package "ROS")
;;//! \htmlinclude TorqueOffset.msg.html


(defclass kortex_driver::TorqueOffset
  :super ros::object
  :slots (_torque_offset ))

(defmethod kortex_driver::TorqueOffset
  (:init
   (&key
    ((:torque_offset __torque_offset) 0.0)
    )
   (send-super :init)
   (setq _torque_offset (float __torque_offset))
   self)
  (:torque_offset
   (&optional __torque_offset)
   (if __torque_offset (setq _torque_offset __torque_offset)) _torque_offset)
  (:serialization-length
   ()
   (+
    ;; float32 _torque_offset
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _torque_offset
       (sys::poke _torque_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _torque_offset
     (setq _torque_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TorqueOffset :md5sum-) "0ac8f77f2604cb21bcfb1cb83c3c05e7")
(setf (get kortex_driver::TorqueOffset :datatype-) "kortex_driver/TorqueOffset")
(setf (get kortex_driver::TorqueOffset :definition-)
      "
float32 torque_offset
")



(provide :kortex_driver/TorqueOffset "0ac8f77f2604cb21bcfb1cb83c3c05e7")


