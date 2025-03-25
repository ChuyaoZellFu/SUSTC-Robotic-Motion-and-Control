;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeviceConfig_CapSenseConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeviceConfig_CapSenseConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DEVICECONFIG_CAPSENSECONFIG")
  (make-package "KORTEX_DRIVER::DEVICECONFIG_CAPSENSECONFIG"))

(in-package "ROS")
;;//! \htmlinclude DeviceConfig_CapSenseConfig.msg.html


(defclass kortex_driver::DeviceConfig_CapSenseConfig
  :super ros::object
  :slots (_mode _threshold_a _threshold_b ))

(defmethod kortex_driver::DeviceConfig_CapSenseConfig
  (:init
   (&key
    ((:mode __mode) 0)
    ((:threshold_a __threshold_a) 0.0)
    ((:threshold_b __threshold_b) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _threshold_a (float __threshold_a))
   (setq _threshold_b (float __threshold_b))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:threshold_a
   (&optional __threshold_a)
   (if __threshold_a (setq _threshold_a __threshold_a)) _threshold_a)
  (:threshold_b
   (&optional __threshold_b)
   (if __threshold_b (setq _threshold_b __threshold_b)) _threshold_b)
  (:serialization-length
   ()
   (+
    ;; uint32 _mode
    4
    ;; float32 _threshold_a
    4
    ;; float32 _threshold_b
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _mode
       (write-long _mode s)
     ;; float32 _threshold_a
       (sys::poke _threshold_a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _threshold_b
       (sys::poke _threshold_b (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _threshold_a
     (setq _threshold_a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _threshold_b
     (setq _threshold_b (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::DeviceConfig_CapSenseConfig :md5sum-) "bd34e36d2242f94fe2b0925f57d7bdbd")
(setf (get kortex_driver::DeviceConfig_CapSenseConfig :datatype-) "kortex_driver/DeviceConfig_CapSenseConfig")
(setf (get kortex_driver::DeviceConfig_CapSenseConfig :definition-)
      "
uint32 mode
float32 threshold_a
float32 threshold_b
")



(provide :kortex_driver/DeviceConfig_CapSenseConfig "bd34e36d2242f94fe2b0925f57d7bdbd")


