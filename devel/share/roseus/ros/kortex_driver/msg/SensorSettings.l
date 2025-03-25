;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SensorSettings)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SensorSettings (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENSORSETTINGS")
  (make-package "KORTEX_DRIVER::SENSORSETTINGS"))

(in-package "ROS")
;;//! \htmlinclude SensorSettings.msg.html


(defclass kortex_driver::SensorSettings
  :super ros::object
  :slots (_sensor _resolution _frame_rate _bit_rate ))

(defmethod kortex_driver::SensorSettings
  (:init
   (&key
    ((:sensor __sensor) 0)
    ((:resolution __resolution) 0)
    ((:frame_rate __frame_rate) 0)
    ((:bit_rate __bit_rate) 0)
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   (setq _resolution (round __resolution))
   (setq _frame_rate (round __frame_rate))
   (setq _bit_rate (round __bit_rate))
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:frame_rate
   (&optional __frame_rate)
   (if __frame_rate (setq _frame_rate __frame_rate)) _frame_rate)
  (:bit_rate
   (&optional __bit_rate)
   (if __bit_rate (setq _bit_rate __bit_rate)) _bit_rate)
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ;; uint32 _resolution
    4
    ;; uint32 _frame_rate
    4
    ;; uint32 _bit_rate
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sensor
       (write-long _sensor s)
     ;; uint32 _resolution
       (write-long _resolution s)
     ;; uint32 _frame_rate
       (write-long _frame_rate s)
     ;; uint32 _bit_rate
       (write-long _bit_rate s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _resolution
     (setq _resolution (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _frame_rate
     (setq _frame_rate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _bit_rate
     (setq _bit_rate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SensorSettings :md5sum-) "1b1f6cd3d6eb9aa0d6912b66a0f489bd")
(setf (get kortex_driver::SensorSettings :datatype-) "kortex_driver/SensorSettings")
(setf (get kortex_driver::SensorSettings :definition-)
      "
uint32 sensor
uint32 resolution
uint32 frame_rate
uint32 bit_rate
")



(provide :kortex_driver/SensorSettings "1b1f6cd3d6eb9aa0d6912b66a0f489bd")


