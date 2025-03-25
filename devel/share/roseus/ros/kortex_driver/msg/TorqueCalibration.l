;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TorqueCalibration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TorqueCalibration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TORQUECALIBRATION")
  (make-package "KORTEX_DRIVER::TORQUECALIBRATION"))

(in-package "ROS")
;;//! \htmlinclude TorqueCalibration.msg.html


(defclass kortex_driver::TorqueCalibration
  :super ros::object
  :slots (_global_gain _global_offset _gain _offset ))

(defmethod kortex_driver::TorqueCalibration
  (:init
   (&key
    ((:global_gain __global_gain) 0.0)
    ((:global_offset __global_offset) 0.0)
    ((:gain __gain) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:offset __offset) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _global_gain (float __global_gain))
   (setq _global_offset (float __global_offset))
   (setq _gain __gain)
   (setq _offset __offset)
   self)
  (:global_gain
   (&optional __global_gain)
   (if __global_gain (setq _global_gain __global_gain)) _global_gain)
  (:global_offset
   (&optional __global_offset)
   (if __global_offset (setq _global_offset __global_offset)) _global_offset)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:offset
   (&optional __offset)
   (if __offset (setq _offset __offset)) _offset)
  (:serialization-length
   ()
   (+
    ;; float32 _global_gain
    4
    ;; float32 _global_offset
    4
    ;; float32[] _gain
    (* 4    (length _gain)) 4
    ;; float32[] _offset
    (* 4    (length _offset)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _global_gain
       (sys::poke _global_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _global_offset
       (sys::poke _global_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _gain
     (write-long (length _gain) s)
     (dotimes (i (length _gain))
       (sys::poke (elt _gain i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _offset
     (write-long (length _offset) s)
     (dotimes (i (length _offset))
       (sys::poke (elt _offset i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _global_gain
     (setq _global_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _global_offset
     (setq _global_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _gain
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gain (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gain i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _offset
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _offset (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _offset i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::TorqueCalibration :md5sum-) "313d945f71d43fe1540908829effe904")
(setf (get kortex_driver::TorqueCalibration :datatype-) "kortex_driver/TorqueCalibration")
(setf (get kortex_driver::TorqueCalibration :definition-)
      "
float32 global_gain
float32 global_offset
float32[] gain
float32[] offset
")



(provide :kortex_driver/TorqueCalibration "313d945f71d43fe1540908829effe904")


