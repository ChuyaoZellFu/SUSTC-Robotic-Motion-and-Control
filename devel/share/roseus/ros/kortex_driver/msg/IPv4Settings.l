;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::IPv4Settings)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'IPv4Settings (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::IPV4SETTINGS")
  (make-package "KORTEX_DRIVER::IPV4SETTINGS"))

(in-package "ROS")
;;//! \htmlinclude IPv4Settings.msg.html


(defclass kortex_driver::IPv4Settings
  :super ros::object
  :slots (_ipv4_address _ipv4_subnet_mask _ipv4_default_gateway ))

(defmethod kortex_driver::IPv4Settings
  (:init
   (&key
    ((:ipv4_address __ipv4_address) 0)
    ((:ipv4_subnet_mask __ipv4_subnet_mask) 0)
    ((:ipv4_default_gateway __ipv4_default_gateway) 0)
    )
   (send-super :init)
   (setq _ipv4_address (round __ipv4_address))
   (setq _ipv4_subnet_mask (round __ipv4_subnet_mask))
   (setq _ipv4_default_gateway (round __ipv4_default_gateway))
   self)
  (:ipv4_address
   (&optional __ipv4_address)
   (if __ipv4_address (setq _ipv4_address __ipv4_address)) _ipv4_address)
  (:ipv4_subnet_mask
   (&optional __ipv4_subnet_mask)
   (if __ipv4_subnet_mask (setq _ipv4_subnet_mask __ipv4_subnet_mask)) _ipv4_subnet_mask)
  (:ipv4_default_gateway
   (&optional __ipv4_default_gateway)
   (if __ipv4_default_gateway (setq _ipv4_default_gateway __ipv4_default_gateway)) _ipv4_default_gateway)
  (:serialization-length
   ()
   (+
    ;; uint32 _ipv4_address
    4
    ;; uint32 _ipv4_subnet_mask
    4
    ;; uint32 _ipv4_default_gateway
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _ipv4_address
       (write-long _ipv4_address s)
     ;; uint32 _ipv4_subnet_mask
       (write-long _ipv4_subnet_mask s)
     ;; uint32 _ipv4_default_gateway
       (write-long _ipv4_default_gateway s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _ipv4_address
     (setq _ipv4_address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ipv4_subnet_mask
     (setq _ipv4_subnet_mask (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ipv4_default_gateway
     (setq _ipv4_default_gateway (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::IPv4Settings :md5sum-) "485e8d7e0822786c2180cac4b25f7434")
(setf (get kortex_driver::IPv4Settings :datatype-) "kortex_driver/IPv4Settings")
(setf (get kortex_driver::IPv4Settings :definition-)
      "
uint32 ipv4_address
uint32 ipv4_subnet_mask
uint32 ipv4_default_gateway
")



(provide :kortex_driver/IPv4Settings "485e8d7e0822786c2180cac4b25f7434")


