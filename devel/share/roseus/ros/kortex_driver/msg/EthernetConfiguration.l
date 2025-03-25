;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EthernetConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EthernetConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ETHERNETCONFIGURATION")
  (make-package "KORTEX_DRIVER::ETHERNETCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude EthernetConfiguration.msg.html


(defclass kortex_driver::EthernetConfiguration
  :super ros::object
  :slots (_device _enabled _speed _duplex ))

(defmethod kortex_driver::EthernetConfiguration
  (:init
   (&key
    ((:device __device) 0)
    ((:enabled __enabled) nil)
    ((:speed __speed) 0)
    ((:duplex __duplex) 0)
    )
   (send-super :init)
   (setq _device (round __device))
   (setq _enabled __enabled)
   (setq _speed (round __speed))
   (setq _duplex (round __duplex))
   self)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:duplex
   (&optional __duplex)
   (if __duplex (setq _duplex __duplex)) _duplex)
  (:serialization-length
   ()
   (+
    ;; uint32 _device
    4
    ;; bool _enabled
    1
    ;; uint32 _speed
    4
    ;; uint32 _duplex
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device
       (write-long _device s)
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _speed
       (write-long _speed s)
     ;; uint32 _duplex
       (write-long _duplex s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _duplex
     (setq _duplex (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::EthernetConfiguration :md5sum-) "f0157b8e3c3c47a25a98debdeb63bdb1")
(setf (get kortex_driver::EthernetConfiguration :datatype-) "kortex_driver/EthernetConfiguration")
(setf (get kortex_driver::EthernetConfiguration :definition-)
      "
uint32 device
bool enabled
uint32 speed
uint32 duplex
")



(provide :kortex_driver/EthernetConfiguration "f0157b8e3c3c47a25a98debdeb63bdb1")


