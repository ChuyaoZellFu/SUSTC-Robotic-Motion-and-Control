;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GpioCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GpioCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOCOMMAND")
  (make-package "KORTEX_DRIVER::GPIOCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude GpioCommand.msg.html


(defclass kortex_driver::GpioCommand
  :super ros::object
  :slots (_port_identifier _pin_identifier _action _period ))

(defmethod kortex_driver::GpioCommand
  (:init
   (&key
    ((:port_identifier __port_identifier) 0)
    ((:pin_identifier __pin_identifier) 0)
    ((:action __action) 0)
    ((:period __period) 0)
    )
   (send-super :init)
   (setq _port_identifier (round __port_identifier))
   (setq _pin_identifier (round __pin_identifier))
   (setq _action (round __action))
   (setq _period (round __period))
   self)
  (:port_identifier
   (&optional __port_identifier)
   (if __port_identifier (setq _port_identifier __port_identifier)) _port_identifier)
  (:pin_identifier
   (&optional __pin_identifier)
   (if __pin_identifier (setq _pin_identifier __pin_identifier)) _pin_identifier)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:serialization-length
   ()
   (+
    ;; uint32 _port_identifier
    4
    ;; uint32 _pin_identifier
    4
    ;; uint32 _action
    4
    ;; uint32 _period
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _port_identifier
       (write-long _port_identifier s)
     ;; uint32 _pin_identifier
       (write-long _pin_identifier s)
     ;; uint32 _action
       (write-long _action s)
     ;; uint32 _period
       (write-long _period s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _port_identifier
     (setq _port_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _pin_identifier
     (setq _pin_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _action
     (setq _action (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _period
     (setq _period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::GpioCommand :md5sum-) "407fd312655ca86f1a6125a8f767d374")
(setf (get kortex_driver::GpioCommand :datatype-) "kortex_driver/GpioCommand")
(setf (get kortex_driver::GpioCommand :definition-)
      "
uint32 port_identifier
uint32 pin_identifier
uint32 action
uint32 period
")



(provide :kortex_driver/GpioCommand "407fd312655ca86f1a6125a8f767d374")


