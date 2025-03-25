;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::IntrinsicProfileIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'IntrinsicProfileIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTRINSICPROFILEIDENTIFIER")
  (make-package "KORTEX_DRIVER::INTRINSICPROFILEIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude IntrinsicProfileIdentifier.msg.html


(defclass kortex_driver::IntrinsicProfileIdentifier
  :super ros::object
  :slots (_sensor _resolution ))

(defmethod kortex_driver::IntrinsicProfileIdentifier
  (:init
   (&key
    ((:sensor __sensor) 0)
    ((:resolution __resolution) 0)
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   (setq _resolution (round __resolution))
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ;; uint32 _resolution
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _resolution
     (setq _resolution (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::IntrinsicProfileIdentifier :md5sum-) "2934edac4ee0b8f88644bc2d0135508a")
(setf (get kortex_driver::IntrinsicProfileIdentifier :datatype-) "kortex_driver/IntrinsicProfileIdentifier")
(setf (get kortex_driver::IntrinsicProfileIdentifier :definition-)
      "
uint32 sensor
uint32 resolution
")



(provide :kortex_driver/IntrinsicProfileIdentifier "2934edac4ee0b8f88644bc2d0135508a")


