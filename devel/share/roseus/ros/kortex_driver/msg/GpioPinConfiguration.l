;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GpioPinConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GpioPinConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOPINCONFIGURATION")
  (make-package "KORTEX_DRIVER::GPIOPINCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude GpioPinConfiguration.msg.html


(defclass kortex_driver::GpioPinConfiguration
  :super ros::object
  :slots (_pin_id _pin_property _output_enable _default_output_value ))

(defmethod kortex_driver::GpioPinConfiguration
  (:init
   (&key
    ((:pin_id __pin_id) 0)
    ((:pin_property __pin_property) 0)
    ((:output_enable __output_enable) nil)
    ((:default_output_value __default_output_value) nil)
    )
   (send-super :init)
   (setq _pin_id (round __pin_id))
   (setq _pin_property (round __pin_property))
   (setq _output_enable __output_enable)
   (setq _default_output_value __default_output_value)
   self)
  (:pin_id
   (&optional __pin_id)
   (if __pin_id (setq _pin_id __pin_id)) _pin_id)
  (:pin_property
   (&optional __pin_property)
   (if __pin_property (setq _pin_property __pin_property)) _pin_property)
  (:output_enable
   (&optional (__output_enable :null))
   (if (not (eq __output_enable :null)) (setq _output_enable __output_enable)) _output_enable)
  (:default_output_value
   (&optional (__default_output_value :null))
   (if (not (eq __default_output_value :null)) (setq _default_output_value __default_output_value)) _default_output_value)
  (:serialization-length
   ()
   (+
    ;; uint32 _pin_id
    4
    ;; uint32 _pin_property
    4
    ;; bool _output_enable
    1
    ;; bool _default_output_value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _pin_id
       (write-long _pin_id s)
     ;; uint32 _pin_property
       (write-long _pin_property s)
     ;; bool _output_enable
       (if _output_enable (write-byte -1 s) (write-byte 0 s))
     ;; bool _default_output_value
       (if _default_output_value (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _pin_id
     (setq _pin_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _pin_property
     (setq _pin_property (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _output_enable
     (setq _output_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _default_output_value
     (setq _default_output_value (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::GpioPinConfiguration :md5sum-) "4b71615f4756e2920864ba411102db09")
(setf (get kortex_driver::GpioPinConfiguration :datatype-) "kortex_driver/GpioPinConfiguration")
(setf (get kortex_driver::GpioPinConfiguration :definition-)
      "
uint32 pin_id
uint32 pin_property
bool output_enable
bool default_output_value
")



(provide :kortex_driver/GpioPinConfiguration "4b71615f4756e2920864ba411102db09")


