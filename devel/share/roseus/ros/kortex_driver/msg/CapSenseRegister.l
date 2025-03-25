;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CapSenseRegister)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CapSenseRegister (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CAPSENSEREGISTER")
  (make-package "KORTEX_DRIVER::CAPSENSEREGISTER"))

(in-package "ROS")
;;//! \htmlinclude CapSenseRegister.msg.html


(defclass kortex_driver::CapSenseRegister
  :super ros::object
  :slots (_address _value ))

(defmethod kortex_driver::CapSenseRegister
  (:init
   (&key
    ((:address __address) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   (setq _value (round __value))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _address
    4
    ;; uint32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _address
       (write-long _address s)
     ;; uint32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CapSenseRegister :md5sum-) "e812c91d734d8142f7286bd1229d1f0d")
(setf (get kortex_driver::CapSenseRegister :datatype-) "kortex_driver/CapSenseRegister")
(setf (get kortex_driver::CapSenseRegister :definition-)
      "
uint32 address
uint32 value
")



(provide :kortex_driver/CapSenseRegister "e812c91d734d8142f7286bd1229d1f0d")


