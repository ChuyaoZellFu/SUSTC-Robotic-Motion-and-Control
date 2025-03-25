;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::IPv4Configuration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'IPv4Configuration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::IPV4CONFIGURATION")
  (make-package "KORTEX_DRIVER::IPV4CONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude IPv4Configuration.msg.html


(defclass kortex_driver::IPv4Configuration
  :super ros::object
  :slots (_ip_address _subnet_mask _default_gateway _dhcp_enabled ))

(defmethod kortex_driver::IPv4Configuration
  (:init
   (&key
    ((:ip_address __ip_address) 0)
    ((:subnet_mask __subnet_mask) 0)
    ((:default_gateway __default_gateway) 0)
    ((:dhcp_enabled __dhcp_enabled) nil)
    )
   (send-super :init)
   (setq _ip_address (round __ip_address))
   (setq _subnet_mask (round __subnet_mask))
   (setq _default_gateway (round __default_gateway))
   (setq _dhcp_enabled __dhcp_enabled)
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
  (:dhcp_enabled
   (&optional (__dhcp_enabled :null))
   (if (not (eq __dhcp_enabled :null)) (setq _dhcp_enabled __dhcp_enabled)) _dhcp_enabled)
  (:serialization-length
   ()
   (+
    ;; uint32 _ip_address
    4
    ;; uint32 _subnet_mask
    4
    ;; uint32 _default_gateway
    4
    ;; bool _dhcp_enabled
    1
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
     ;; bool _dhcp_enabled
       (if _dhcp_enabled (write-byte -1 s) (write-byte 0 s))
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
   ;; bool _dhcp_enabled
     (setq _dhcp_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::IPv4Configuration :md5sum-) "938bacc92850e7d808a96d82d5b98e81")
(setf (get kortex_driver::IPv4Configuration :datatype-) "kortex_driver/IPv4Configuration")
(setf (get kortex_driver::IPv4Configuration :definition-)
      "
uint32 ip_address
uint32 subnet_mask
uint32 default_gateway
bool dhcp_enabled
")



(provide :kortex_driver/IPv4Configuration "938bacc92850e7d808a96d82d5b98e81")


