;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EmergencyStop)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EmergencyStop (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::EMERGENCYSTOP")
  (make-package "KORTEX_DRIVER::EMERGENCYSTOP"))

(in-package "ROS")
;;//! \htmlinclude EmergencyStop.msg.html


(defclass kortex_driver::EmergencyStop
  :super ros::object
  :slots ())

(defmethod kortex_driver::EmergencyStop
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::EmergencyStop :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get kortex_driver::EmergencyStop :datatype-) "kortex_driver/EmergencyStop")
(setf (get kortex_driver::EmergencyStop :definition-)
      "
")



(provide :kortex_driver/EmergencyStop "d41d8cd98f00b204e9800998ecf8427e")


