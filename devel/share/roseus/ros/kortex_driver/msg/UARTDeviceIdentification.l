;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UARTDeviceIdentification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UARTDeviceIdentification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UARTDEVICEIDENTIFICATION")
  (make-package "KORTEX_DRIVER::UARTDEVICEIDENTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude UARTDeviceIdentification.msg.html


(defclass kortex_driver::UARTDeviceIdentification
  :super ros::object
  :slots (_port_id ))

(defmethod kortex_driver::UARTDeviceIdentification
  (:init
   (&key
    ((:port_id __port_id) 0)
    )
   (send-super :init)
   (setq _port_id (round __port_id))
   self)
  (:port_id
   (&optional __port_id)
   (if __port_id (setq _port_id __port_id)) _port_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _port_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _port_id
       (write-long _port_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _port_id
     (setq _port_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::UARTDeviceIdentification :md5sum-) "c688e9b638ff5d4cab989955c0b36eb8")
(setf (get kortex_driver::UARTDeviceIdentification :datatype-) "kortex_driver/UARTDeviceIdentification")
(setf (get kortex_driver::UARTDeviceIdentification :definition-)
      "
uint32 port_id
")



(provide :kortex_driver/UARTDeviceIdentification "c688e9b638ff5d4cab989955c0b36eb8")


