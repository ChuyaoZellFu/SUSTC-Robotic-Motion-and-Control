;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_CapSenseConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_CapSenseConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_CAPSENSECONFIG")
  (make-package "KORTEX_DRIVER::BASE_CAPSENSECONFIG"))

(in-package "ROS")
;;//! \htmlinclude Base_CapSenseConfig.msg.html


(defclass kortex_driver::Base_CapSenseConfig
  :super ros::object
  :slots (_identifier _mode _threshold_a _threshold_b _sensitivity_a _sensitivity_b ))

(defmethod kortex_driver::Base_CapSenseConfig
  (:init
   (&key
    ((:identifier __identifier) 0)
    ((:mode __mode) 0)
    ((:threshold_a __threshold_a) 0.0)
    ((:threshold_b __threshold_b) 0.0)
    ((:sensitivity_a __sensitivity_a) 0.0)
    ((:sensitivity_b __sensitivity_b) 0.0)
    )
   (send-super :init)
   (setq _identifier (round __identifier))
   (setq _mode (round __mode))
   (setq _threshold_a (float __threshold_a))
   (setq _threshold_b (float __threshold_b))
   (setq _sensitivity_a (float __sensitivity_a))
   (setq _sensitivity_b (float __sensitivity_b))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:threshold_a
   (&optional __threshold_a)
   (if __threshold_a (setq _threshold_a __threshold_a)) _threshold_a)
  (:threshold_b
   (&optional __threshold_b)
   (if __threshold_b (setq _threshold_b __threshold_b)) _threshold_b)
  (:sensitivity_a
   (&optional __sensitivity_a)
   (if __sensitivity_a (setq _sensitivity_a __sensitivity_a)) _sensitivity_a)
  (:sensitivity_b
   (&optional __sensitivity_b)
   (if __sensitivity_b (setq _sensitivity_b __sensitivity_b)) _sensitivity_b)
  (:serialization-length
   ()
   (+
    ;; uint32 _identifier
    4
    ;; uint32 _mode
    4
    ;; float32 _threshold_a
    4
    ;; float32 _threshold_b
    4
    ;; float32 _sensitivity_a
    4
    ;; float32 _sensitivity_b
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _identifier
       (write-long _identifier s)
     ;; uint32 _mode
       (write-long _mode s)
     ;; float32 _threshold_a
       (sys::poke _threshold_a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _threshold_b
       (sys::poke _threshold_b (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sensitivity_a
       (sys::poke _sensitivity_a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sensitivity_b
       (sys::poke _sensitivity_b (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _identifier
     (setq _identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _threshold_a
     (setq _threshold_a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _threshold_b
     (setq _threshold_b (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sensitivity_a
     (setq _sensitivity_a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sensitivity_b
     (setq _sensitivity_b (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Base_CapSenseConfig :md5sum-) "f62e507319ac72b37a316fe4248e2e5a")
(setf (get kortex_driver::Base_CapSenseConfig :datatype-) "kortex_driver/Base_CapSenseConfig")
(setf (get kortex_driver::Base_CapSenseConfig :definition-)
      "
uint32 identifier
uint32 mode
float32 threshold_a
float32 threshold_b
float32 sensitivity_a
float32 sensitivity_b
")



(provide :kortex_driver/Base_CapSenseConfig "f62e507319ac72b37a316fe4248e2e5a")


