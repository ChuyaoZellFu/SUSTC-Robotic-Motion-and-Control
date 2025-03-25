;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORINFORMATION")
  (make-package "KORTEX_DRIVER::ACTUATORINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude ActuatorInformation.msg.html


(defclass kortex_driver::ActuatorInformation
  :super ros::object
  :slots (_count ))

(defmethod kortex_driver::ActuatorInformation
  (:init
   (&key
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _count (round __count))
   self)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; uint32 _count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _count
       (write-long _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ActuatorInformation :md5sum-) "ac8b22eb02c1f433e0a55ee9aac59a18")
(setf (get kortex_driver::ActuatorInformation :datatype-) "kortex_driver/ActuatorInformation")
(setf (get kortex_driver::ActuatorInformation :definition-)
      "
uint32 count
")



(provide :kortex_driver/ActuatorInformation "ac8b22eb02c1f433e0a55ee9aac59a18")


