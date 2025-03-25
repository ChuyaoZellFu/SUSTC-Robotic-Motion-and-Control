;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ManualFocus)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ManualFocus (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MANUALFOCUS")
  (make-package "KORTEX_DRIVER::MANUALFOCUS"))

(in-package "ROS")
;;//! \htmlinclude ManualFocus.msg.html


(defclass kortex_driver::ManualFocus
  :super ros::object
  :slots (_value ))

(defmethod kortex_driver::ManualFocus
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

(setf (get kortex_driver::ManualFocus :md5sum-) "d9d213e8c146065ec958333c617eb1a6")
(setf (get kortex_driver::ManualFocus :datatype-) "kortex_driver/ManualFocus")
(setf (get kortex_driver::ManualFocus :definition-)
      "
uint32 value
")



(provide :kortex_driver/ManualFocus "d9d213e8c146065ec958333c617eb1a6")


