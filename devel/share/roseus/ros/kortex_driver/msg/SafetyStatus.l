;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyStatus)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyStatus (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYSTATUS")
  (make-package "KORTEX_DRIVER::SAFETYSTATUS"))

(in-package "ROS")
;;//! \htmlinclude SafetyStatus.msg.html


(defclass kortex_driver::SafetyStatus
  :super ros::object
  :slots (_value ))

(defmethod kortex_driver::SafetyStatus
  (:init
   (&key
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _value (round __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SafetyStatus :md5sum-) "d9d213e8c146065ec958333c617eb1a6")
(setf (get kortex_driver::SafetyStatus :datatype-) "kortex_driver/SafetyStatus")
(setf (get kortex_driver::SafetyStatus :definition-)
      "
uint32 value
")



(provide :kortex_driver/SafetyStatus "d9d213e8c146065ec958333c617eb1a6")


