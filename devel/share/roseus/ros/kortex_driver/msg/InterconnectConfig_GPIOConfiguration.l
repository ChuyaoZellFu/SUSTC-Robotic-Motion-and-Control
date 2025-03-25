;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectConfig_GPIOConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectConfig_GPIOConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCONFIG_GPIOCONFIGURATION")
  (make-package "KORTEX_DRIVER::INTERCONNECTCONFIG_GPIOCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude InterconnectConfig_GPIOConfiguration.msg.html


(defclass kortex_driver::InterconnectConfig_GPIOConfiguration
  :super ros::object
  :slots (_identifier _mode _pull _default_value ))

(defmethod kortex_driver::InterconnectConfig_GPIOConfiguration
  (:init
   (&key
    ((:identifier __identifier) 0)
    ((:mode __mode) 0)
    ((:pull __pull) 0)
    ((:default_value __default_value) 0)
    )
   (send-super :init)
   (setq _identifier (round __identifier))
   (setq _mode (round __mode))
   (setq _pull (round __pull))
   (setq _default_value (round __default_value))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:pull
   (&optional __pull)
   (if __pull (setq _pull __pull)) _pull)
  (:default_value
   (&optional __default_value)
   (if __default_value (setq _default_value __default_value)) _default_value)
  (:serialization-length
   ()
   (+
    ;; uint32 _identifier
    4
    ;; uint32 _mode
    4
    ;; uint32 _pull
    4
    ;; uint32 _default_value
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
     ;; uint32 _pull
       (write-long _pull s)
     ;; uint32 _default_value
       (write-long _default_value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _identifier
     (setq _identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _pull
     (setq _pull (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _default_value
     (setq _default_value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::InterconnectConfig_GPIOConfiguration :md5sum-) "beac18d5b1b9bb04dc5a1a2c0d366dac")
(setf (get kortex_driver::InterconnectConfig_GPIOConfiguration :datatype-) "kortex_driver/InterconnectConfig_GPIOConfiguration")
(setf (get kortex_driver::InterconnectConfig_GPIOConfiguration :definition-)
      "
uint32 identifier
uint32 mode
uint32 pull
uint32 default_value
")



(provide :kortex_driver/InterconnectConfig_GPIOConfiguration "beac18d5b1b9bb04dc5a1a2c0d366dac")


