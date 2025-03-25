;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLEREVENT")
  (make-package "KORTEX_DRIVER::CONTROLLEREVENT"))

(in-package "ROS")
;;//! \htmlinclude ControllerEvent.msg.html


(defclass kortex_driver::ControllerEvent
  :super ros::object
  :slots (_input_type _behavior _input_identifier ))

(defmethod kortex_driver::ControllerEvent
  (:init
   (&key
    ((:input_type __input_type) 0)
    ((:behavior __behavior) 0)
    ((:input_identifier __input_identifier) 0)
    )
   (send-super :init)
   (setq _input_type (round __input_type))
   (setq _behavior (round __behavior))
   (setq _input_identifier (round __input_identifier))
   self)
  (:input_type
   (&optional __input_type)
   (if __input_type (setq _input_type __input_type)) _input_type)
  (:behavior
   (&optional __behavior)
   (if __behavior (setq _behavior __behavior)) _behavior)
  (:input_identifier
   (&optional __input_identifier)
   (if __input_identifier (setq _input_identifier __input_identifier)) _input_identifier)
  (:serialization-length
   ()
   (+
    ;; uint32 _input_type
    4
    ;; uint32 _behavior
    4
    ;; uint32 _input_identifier
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _input_type
       (write-long _input_type s)
     ;; uint32 _behavior
       (write-long _behavior s)
     ;; uint32 _input_identifier
       (write-long _input_identifier s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _input_type
     (setq _input_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _behavior
     (setq _behavior (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _input_identifier
     (setq _input_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ControllerEvent :md5sum-) "a8f7ada35fd120a9401b9d95b206763c")
(setf (get kortex_driver::ControllerEvent :datatype-) "kortex_driver/ControllerEvent")
(setf (get kortex_driver::ControllerEvent :definition-)
      "
uint32 input_type
uint32 behavior
uint32 input_identifier
")



(provide :kortex_driver/ControllerEvent "a8f7ada35fd120a9401b9d95b206763c")


