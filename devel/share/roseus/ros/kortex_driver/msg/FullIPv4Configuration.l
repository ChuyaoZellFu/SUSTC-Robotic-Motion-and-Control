;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FullIPv4Configuration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FullIPv4Configuration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FULLIPV4CONFIGURATION")
  (make-package "KORTEX_DRIVER::FULLIPV4CONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude FullIPv4Configuration.msg.html


(defclass kortex_driver::FullIPv4Configuration
  :super ros::object
  :slots (_handle _ipv4_configuration ))

(defmethod kortex_driver::FullIPv4Configuration
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::NetworkHandle :init))
    ((:ipv4_configuration __ipv4_configuration) (instance kortex_driver::IPv4Configuration :init))
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _ipv4_configuration __ipv4_configuration)
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:ipv4_configuration
   (&rest __ipv4_configuration)
   (if (keywordp (car __ipv4_configuration))
       (send* _ipv4_configuration __ipv4_configuration)
     (progn
       (if __ipv4_configuration (setq _ipv4_configuration (car __ipv4_configuration)))
       _ipv4_configuration)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/NetworkHandle _handle
    (send _handle :serialization-length)
    ;; kortex_driver/IPv4Configuration _ipv4_configuration
    (send _ipv4_configuration :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/NetworkHandle _handle
       (send _handle :serialize s)
     ;; kortex_driver/IPv4Configuration _ipv4_configuration
       (send _ipv4_configuration :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/NetworkHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; kortex_driver/IPv4Configuration _ipv4_configuration
     (send _ipv4_configuration :deserialize buf ptr-) (incf ptr- (send _ipv4_configuration :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::FullIPv4Configuration :md5sum-) "b0ba6974979f83d97757b00aa8a1bc7d")
(setf (get kortex_driver::FullIPv4Configuration :datatype-) "kortex_driver/FullIPv4Configuration")
(setf (get kortex_driver::FullIPv4Configuration :definition-)
      "
NetworkHandle handle
IPv4Configuration ipv4_configuration
================================================================================
MSG: kortex_driver/NetworkHandle

uint32 type
================================================================================
MSG: kortex_driver/IPv4Configuration

uint32 ip_address
uint32 subnet_mask
uint32 default_gateway
bool dhcp_enabled
")



(provide :kortex_driver/FullIPv4Configuration "b0ba6974979f83d97757b00aa8a1bc7d")


