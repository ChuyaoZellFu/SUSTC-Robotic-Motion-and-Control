;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::NotificationOptions)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'NotificationOptions (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::NOTIFICATIONOPTIONS")
  (make-package "KORTEX_DRIVER::NOTIFICATIONOPTIONS"))

(in-package "ROS")
;;//! \htmlinclude NotificationOptions.msg.html


(defclass kortex_driver::NotificationOptions
  :super ros::object
  :slots (_type _rate_m_sec _threshold_value ))

(defmethod kortex_driver::NotificationOptions
  (:init
   (&key
    ((:type __type) 0)
    ((:rate_m_sec __rate_m_sec) 0)
    ((:threshold_value __threshold_value) 0.0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _rate_m_sec (round __rate_m_sec))
   (setq _threshold_value (float __threshold_value))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:rate_m_sec
   (&optional __rate_m_sec)
   (if __rate_m_sec (setq _rate_m_sec __rate_m_sec)) _rate_m_sec)
  (:threshold_value
   (&optional __threshold_value)
   (if __threshold_value (setq _threshold_value __threshold_value)) _threshold_value)
  (:serialization-length
   ()
   (+
    ;; uint32 _type
    4
    ;; uint32 _rate_m_sec
    4
    ;; float32 _threshold_value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _type
       (write-long _type s)
     ;; uint32 _rate_m_sec
       (write-long _rate_m_sec s)
     ;; float32 _threshold_value
       (sys::poke _threshold_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _rate_m_sec
     (setq _rate_m_sec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _threshold_value
     (setq _threshold_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::NotificationOptions :md5sum-) "756dde28efcdf67e7515ead30453e781")
(setf (get kortex_driver::NotificationOptions :datatype-) "kortex_driver/NotificationOptions")
(setf (get kortex_driver::NotificationOptions :definition-)
      "
uint32 type
uint32 rate_m_sec
float32 threshold_value
")



(provide :kortex_driver/NotificationOptions "756dde28efcdf67e7515ead30453e781")


