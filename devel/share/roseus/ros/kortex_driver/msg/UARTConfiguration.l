;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UARTConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UARTConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UARTCONFIGURATION")
  (make-package "KORTEX_DRIVER::UARTCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude UARTConfiguration.msg.html


(defclass kortex_driver::UARTConfiguration
  :super ros::object
  :slots (_port_id _enabled _speed _word_length _stop_bits _parity ))

(defmethod kortex_driver::UARTConfiguration
  (:init
   (&key
    ((:port_id __port_id) 0)
    ((:enabled __enabled) nil)
    ((:speed __speed) 0)
    ((:word_length __word_length) 0)
    ((:stop_bits __stop_bits) 0)
    ((:parity __parity) 0)
    )
   (send-super :init)
   (setq _port_id (round __port_id))
   (setq _enabled __enabled)
   (setq _speed (round __speed))
   (setq _word_length (round __word_length))
   (setq _stop_bits (round __stop_bits))
   (setq _parity (round __parity))
   self)
  (:port_id
   (&optional __port_id)
   (if __port_id (setq _port_id __port_id)) _port_id)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:word_length
   (&optional __word_length)
   (if __word_length (setq _word_length __word_length)) _word_length)
  (:stop_bits
   (&optional __stop_bits)
   (if __stop_bits (setq _stop_bits __stop_bits)) _stop_bits)
  (:parity
   (&optional __parity)
   (if __parity (setq _parity __parity)) _parity)
  (:serialization-length
   ()
   (+
    ;; uint32 _port_id
    4
    ;; bool _enabled
    1
    ;; uint32 _speed
    4
    ;; uint32 _word_length
    4
    ;; uint32 _stop_bits
    4
    ;; uint32 _parity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _port_id
       (write-long _port_id s)
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _speed
       (write-long _speed s)
     ;; uint32 _word_length
       (write-long _word_length s)
     ;; uint32 _stop_bits
       (write-long _stop_bits s)
     ;; uint32 _parity
       (write-long _parity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _port_id
     (setq _port_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _word_length
     (setq _word_length (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _stop_bits
     (setq _stop_bits (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _parity
     (setq _parity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::UARTConfiguration :md5sum-) "da8e919001edf0734e1795af994fc4e9")
(setf (get kortex_driver::UARTConfiguration :datatype-) "kortex_driver/UARTConfiguration")
(setf (get kortex_driver::UARTConfiguration :definition-)
      "
uint32 port_id
bool enabled
uint32 speed
uint32 word_length
uint32 stop_bits
uint32 parity
")



(provide :kortex_driver/UARTConfiguration "da8e919001edf0734e1795af994fc4e9")


