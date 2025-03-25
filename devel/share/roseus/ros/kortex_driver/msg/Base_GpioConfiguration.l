;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_GpioConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_GpioConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_GPIOCONFIGURATION")
  (make-package "KORTEX_DRIVER::BASE_GPIOCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude Base_GpioConfiguration.msg.html


(defclass kortex_driver::Base_GpioConfiguration
  :super ros::object
  :slots (_port_number _pin_configurations ))

(defmethod kortex_driver::Base_GpioConfiguration
  (:init
   (&key
    ((:port_number __port_number) 0)
    ((:pin_configurations __pin_configurations) ())
    )
   (send-super :init)
   (setq _port_number (round __port_number))
   (setq _pin_configurations __pin_configurations)
   self)
  (:port_number
   (&optional __port_number)
   (if __port_number (setq _port_number __port_number)) _port_number)
  (:pin_configurations
   (&rest __pin_configurations)
   (if (keywordp (car __pin_configurations))
       (send* _pin_configurations __pin_configurations)
     (progn
       (if __pin_configurations (setq _pin_configurations (car __pin_configurations)))
       _pin_configurations)))
  (:serialization-length
   ()
   (+
    ;; uint32 _port_number
    4
    ;; kortex_driver/GpioPinConfiguration[] _pin_configurations
    (apply #'+ (send-all _pin_configurations :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _port_number
       (write-long _port_number s)
     ;; kortex_driver/GpioPinConfiguration[] _pin_configurations
     (write-long (length _pin_configurations) s)
     (dolist (elem _pin_configurations)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _port_number
     (setq _port_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/GpioPinConfiguration[] _pin_configurations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pin_configurations (let (r) (dotimes (i n) (push (instance kortex_driver::GpioPinConfiguration :init) r)) r))
     (dolist (elem- _pin_configurations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::Base_GpioConfiguration :md5sum-) "92f1de7cd4a1d8641179ab50f182b3f3")
(setf (get kortex_driver::Base_GpioConfiguration :datatype-) "kortex_driver/Base_GpioConfiguration")
(setf (get kortex_driver::Base_GpioConfiguration :definition-)
      "
uint32 port_number
GpioPinConfiguration[] pin_configurations
================================================================================
MSG: kortex_driver/GpioPinConfiguration

uint32 pin_id
uint32 pin_property
bool output_enable
bool default_output_value
")



(provide :kortex_driver/Base_GpioConfiguration "92f1de7cd4a1d8641179ab50f182b3f3")


