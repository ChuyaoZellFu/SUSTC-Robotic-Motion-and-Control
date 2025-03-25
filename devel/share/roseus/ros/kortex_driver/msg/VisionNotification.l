;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::VisionNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'VisionNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::VISIONNOTIFICATION")
  (make-package "KORTEX_DRIVER::VISIONNOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude VisionNotification.msg.html


(defclass kortex_driver::VisionNotification
  :super ros::object
  :slots (_event _sensor _option ))

(defmethod kortex_driver::VisionNotification
  (:init
   (&key
    ((:event __event) 0)
    ((:sensor __sensor) 0)
    ((:option __option) 0)
    )
   (send-super :init)
   (setq _event (round __event))
   (setq _sensor (round __sensor))
   (setq _option (round __option))
   self)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:option
   (&optional __option)
   (if __option (setq _option __option)) _option)
  (:serialization-length
   ()
   (+
    ;; uint32 _event
    4
    ;; uint32 _sensor
    4
    ;; uint32 _option
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _event
       (write-long _event s)
     ;; uint32 _sensor
       (write-long _sensor s)
     ;; uint32 _option
       (write-long _option s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _event
     (setq _event (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _option
     (setq _option (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::VisionNotification :md5sum-) "1894e81fd5e3bdb5e082271dc8a09dcd")
(setf (get kortex_driver::VisionNotification :datatype-) "kortex_driver/VisionNotification")
(setf (get kortex_driver::VisionNotification :definition-)
      "
uint32 event
uint32 sensor
uint32 option
")



(provide :kortex_driver/VisionNotification "1894e81fd5e3bdb5e082271dc8a09dcd")


