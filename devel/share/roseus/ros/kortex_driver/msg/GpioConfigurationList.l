;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GpioConfigurationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GpioConfigurationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOCONFIGURATIONLIST")
  (make-package "KORTEX_DRIVER::GPIOCONFIGURATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude GpioConfigurationList.msg.html


(defclass kortex_driver::GpioConfigurationList
  :super ros::object
  :slots (_port_configurations ))

(defmethod kortex_driver::GpioConfigurationList
  (:init
   (&key
    ((:port_configurations __port_configurations) ())
    )
   (send-super :init)
   (setq _port_configurations __port_configurations)
   self)
  (:port_configurations
   (&rest __port_configurations)
   (if (keywordp (car __port_configurations))
       (send* _port_configurations __port_configurations)
     (progn
       (if __port_configurations (setq _port_configurations (car __port_configurations)))
       _port_configurations)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Base_GpioConfiguration[] _port_configurations
    (apply #'+ (send-all _port_configurations :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_GpioConfiguration[] _port_configurations
     (write-long (length _port_configurations) s)
     (dolist (elem _port_configurations)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_GpioConfiguration[] _port_configurations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _port_configurations (let (r) (dotimes (i n) (push (instance kortex_driver::Base_GpioConfiguration :init) r)) r))
     (dolist (elem- _port_configurations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::GpioConfigurationList :md5sum-) "2692838fc0bc85259f7645a61387ad92")
(setf (get kortex_driver::GpioConfigurationList :datatype-) "kortex_driver/GpioConfigurationList")
(setf (get kortex_driver::GpioConfigurationList :definition-)
      "
Base_GpioConfiguration[] port_configurations
================================================================================
MSG: kortex_driver/Base_GpioConfiguration

uint32 port_number
GpioPinConfiguration[] pin_configurations
================================================================================
MSG: kortex_driver/GpioPinConfiguration

uint32 pin_id
uint32 pin_property
bool output_enable
bool default_output_value
")



(provide :kortex_driver/GpioConfigurationList "2692838fc0bc85259f7645a61387ad92")


