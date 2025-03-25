;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SensorIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SensorIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENSORIDENTIFIER")
  (make-package "KORTEX_DRIVER::SENSORIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude SensorIdentifier.msg.html


(defclass kortex_driver::SensorIdentifier
  :super ros::object
  :slots (_sensor ))

(defmethod kortex_driver::SensorIdentifier
  (:init
   (&key
    ((:sensor __sensor) 0)
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sensor
       (write-long _sensor s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SensorIdentifier :md5sum-) "b9fe73419757e1a59473fcd91b011ce9")
(setf (get kortex_driver::SensorIdentifier :datatype-) "kortex_driver/SensorIdentifier")
(setf (get kortex_driver::SensorIdentifier :definition-)
      "
uint32 sensor
")



(provide :kortex_driver/SensorIdentifier "b9fe73419757e1a59473fcd91b011ce9")


