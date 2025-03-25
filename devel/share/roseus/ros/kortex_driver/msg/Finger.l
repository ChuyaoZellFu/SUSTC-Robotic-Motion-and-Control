;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Finger)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Finger (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FINGER")
  (make-package "KORTEX_DRIVER::FINGER"))

(in-package "ROS")
;;//! \htmlinclude Finger.msg.html


(defclass kortex_driver::Finger
  :super ros::object
  :slots (_finger_identifier _value ))

(defmethod kortex_driver::Finger
  (:init
   (&key
    ((:finger_identifier __finger_identifier) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _finger_identifier (round __finger_identifier))
   (setq _value (float __value))
   self)
  (:finger_identifier
   (&optional __finger_identifier)
   (if __finger_identifier (setq _finger_identifier __finger_identifier)) _finger_identifier)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _finger_identifier
    4
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _finger_identifier
       (write-long _finger_identifier s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _finger_identifier
     (setq _finger_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Finger :md5sum-) "5bf3287ad42d5a92136c7d9cf8d0eada")
(setf (get kortex_driver::Finger :datatype-) "kortex_driver/Finger")
(setf (get kortex_driver::Finger :definition-)
      "
uint32 finger_identifier
float32 value
")



(provide :kortex_driver/Finger "5bf3287ad42d5a92136c7d9cf8d0eada")


