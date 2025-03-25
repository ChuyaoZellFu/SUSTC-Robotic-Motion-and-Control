;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::IPv4Information)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'IPv4Information (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::IPV4INFORMATION")
  (make-package "KORTEX_DRIVER::IPV4INFORMATION"))

(in-package "ROS")
;;//! \htmlinclude IPv4Information.msg.html


(defclass kortex_driver::IPv4Information
  :super ros::object
  :slots (_ip_address _subnet_mask _default_gateway ))

(defmethod kortex_driver::IPv4Information
  (:init
   (&key
    ((:ip_address __ip_address) 0)
    ((:subnet_mask __subnet_mask) 0)
    ((:default_gateway __default_gateway) 0)
    )
   (send-super :init)
   (setq _ip_address (round __ip_address))
   (setq _subnet_mask (round __subnet_mask))
   (setq _default_gateway (round __default_gateway))
   self)
  (:ip_address
   (&optional __ip_address)
   (if __ip_address (setq _ip_address __ip_address)) _ip_address)
  (:subnet_mask
   (&optional __subnet_mask)
   (if __subnet_mask (setq _subnet_mask __subnet_mask)) _subnet_mask)
  (:default_gateway
   (&optional __default_gateway)
   (if __default_gateway (setq _default_gateway __default_gateway)) _default_gateway)
  (:serialization-length
   ()
   (+
    ;; uint32 _ip_address
    4
    ;; uint32 _subnet_mask
    4
    ;; uint32 _default_gateway
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _ip_address
       (write-long _ip_address s)
     ;; uint32 _subnet_mask
       (write-long _subnet_mask s)
     ;; uint32 _default_gateway
       (write-long _default_gateway s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _ip_address
     (setq _ip_address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _subnet_mask
     (setq _subnet_mask (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _default_gateway
     (setq _default_gateway (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::IPv4Information :md5sum-) "8070c0574c2b6ebcf5c917ce0b71133c")
(setf (get kortex_driver::IPv4Information :datatype-) "kortex_driver/IPv4Information")
(setf (get kortex_driver::IPv4Information :definition-)
      "
uint32 ip_address
uint32 subnet_mask
uint32 default_gateway
")



(provide :kortex_driver/IPv4Information "8070c0574c2b6ebcf5c917ce0b71133c")


