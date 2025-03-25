;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GPIOState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GPIOState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOSTATE")
  (make-package "KORTEX_DRIVER::GPIOSTATE"))

(in-package "ROS")
;;//! \htmlinclude GPIOState.msg.html


(defclass kortex_driver::GPIOState
  :super ros::object
  :slots (_identifier _value ))

(defmethod kortex_driver::GPIOState
  (:init
   (&key
    ((:identifier __identifier) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _identifier (round __identifier))
   (setq _value (round __value))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _identifier
    4
    ;; uint32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _identifier
       (write-long _identifier s)
     ;; uint32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _identifier
     (setq _identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::GPIOState :md5sum-) "8c1bfa5d5edbf20179de7fa3e9a203b2")
(setf (get kortex_driver::GPIOState :datatype-) "kortex_driver/GPIOState")
(setf (get kortex_driver::GPIOState :definition-)
      "
uint32 identifier
uint32 value
")



(provide :kortex_driver/GPIOState "8c1bfa5d5edbf20179de7fa3e9a203b2")


