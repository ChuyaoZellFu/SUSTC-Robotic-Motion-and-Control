;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BridgeConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BridgeConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRIDGECONFIG")
  (make-package "KORTEX_DRIVER::BRIDGECONFIG"))

(in-package "ROS")
;;//! \htmlinclude BridgeConfig.msg.html


(defclass kortex_driver::BridgeConfig
  :super ros::object
  :slots (_device_identifier _bridgetype _port_config _bridge_id ))

(defmethod kortex_driver::BridgeConfig
  (:init
   (&key
    ((:device_identifier __device_identifier) 0)
    ((:bridgetype __bridgetype) 0)
    ((:port_config __port_config) (instance kortex_driver::BridgePortConfig :init))
    ((:bridge_id __bridge_id) (instance kortex_driver::BridgeIdentifier :init))
    )
   (send-super :init)
   (setq _device_identifier (round __device_identifier))
   (setq _bridgetype (round __bridgetype))
   (setq _port_config __port_config)
   (setq _bridge_id __bridge_id)
   self)
  (:device_identifier
   (&optional __device_identifier)
   (if __device_identifier (setq _device_identifier __device_identifier)) _device_identifier)
  (:bridgetype
   (&optional __bridgetype)
   (if __bridgetype (setq _bridgetype __bridgetype)) _bridgetype)
  (:port_config
   (&rest __port_config)
   (if (keywordp (car __port_config))
       (send* _port_config __port_config)
     (progn
       (if __port_config (setq _port_config (car __port_config)))
       _port_config)))
  (:bridge_id
   (&rest __bridge_id)
   (if (keywordp (car __bridge_id))
       (send* _bridge_id __bridge_id)
     (progn
       (if __bridge_id (setq _bridge_id (car __bridge_id)))
       _bridge_id)))
  (:serialization-length
   ()
   (+
    ;; uint32 _device_identifier
    4
    ;; uint32 _bridgetype
    4
    ;; kortex_driver/BridgePortConfig _port_config
    (send _port_config :serialization-length)
    ;; kortex_driver/BridgeIdentifier _bridge_id
    (send _bridge_id :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device_identifier
       (write-long _device_identifier s)
     ;; uint32 _bridgetype
       (write-long _bridgetype s)
     ;; kortex_driver/BridgePortConfig _port_config
       (send _port_config :serialize s)
     ;; kortex_driver/BridgeIdentifier _bridge_id
       (send _bridge_id :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device_identifier
     (setq _device_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _bridgetype
     (setq _bridgetype (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/BridgePortConfig _port_config
     (send _port_config :deserialize buf ptr-) (incf ptr- (send _port_config :serialization-length))
   ;; kortex_driver/BridgeIdentifier _bridge_id
     (send _bridge_id :deserialize buf ptr-) (incf ptr- (send _bridge_id :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::BridgeConfig :md5sum-) "967c7a78caf96ff069310456674faf6a")
(setf (get kortex_driver::BridgeConfig :datatype-) "kortex_driver/BridgeConfig")
(setf (get kortex_driver::BridgeConfig :definition-)
      "
uint32 device_identifier
uint32 bridgetype
BridgePortConfig port_config
BridgeIdentifier bridge_id
================================================================================
MSG: kortex_driver/BridgePortConfig

uint32 target_port
uint32 out_port
================================================================================
MSG: kortex_driver/BridgeIdentifier

uint32 bridge_id
")



(provide :kortex_driver/BridgeConfig "967c7a78caf96ff069310456674faf6a")


