;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Sensor)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Sensor (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENSOR")
  (make-package "KORTEX_DRIVER::SENSOR"))

(in-package "ROS")
;;//! \htmlinclude Sensor.msg.html


(intern "*SENSOR_UNSPECIFIED*" (find-package "KORTEX_DRIVER::SENSOR"))
(shadow '*SENSOR_UNSPECIFIED* (find-package "KORTEX_DRIVER::SENSOR"))
(defconstant kortex_driver::Sensor::*SENSOR_UNSPECIFIED* 0)
(intern "*SENSOR_COLOR*" (find-package "KORTEX_DRIVER::SENSOR"))
(shadow '*SENSOR_COLOR* (find-package "KORTEX_DRIVER::SENSOR"))
(defconstant kortex_driver::Sensor::*SENSOR_COLOR* 1)
(intern "*SENSOR_DEPTH*" (find-package "KORTEX_DRIVER::SENSOR"))
(shadow '*SENSOR_DEPTH* (find-package "KORTEX_DRIVER::SENSOR"))
(defconstant kortex_driver::Sensor::*SENSOR_DEPTH* 2)

(defun kortex_driver::Sensor-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Sensor::*SENSOR_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::Sensor::*SENSOR_COLOR*)
        ((= const 2) 'kortex_driver::Sensor::*SENSOR_DEPTH*)
        (t nil)))

(defclass kortex_driver::Sensor
  :super ros::object
  :slots ())

(defmethod kortex_driver::Sensor
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

(setf (get kortex_driver::Sensor :md5sum-) "4dd10d615f1a44376aea67ca8eb604d0")
(setf (get kortex_driver::Sensor :datatype-) "kortex_driver/Sensor")
(setf (get kortex_driver::Sensor :definition-)
      "
uint32 SENSOR_UNSPECIFIED = 0

uint32 SENSOR_COLOR = 1

uint32 SENSOR_DEPTH = 2

")



(provide :kortex_driver/Sensor "4dd10d615f1a44376aea67ca8eb604d0")


