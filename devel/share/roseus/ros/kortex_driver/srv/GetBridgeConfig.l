;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetBridgeConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetBridgeConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETBRIDGECONFIG")
  (make-package "KORTEX_DRIVER::GETBRIDGECONFIG"))
(unless (find-package "KORTEX_DRIVER::GETBRIDGECONFIGREQUEST")
  (make-package "KORTEX_DRIVER::GETBRIDGECONFIGREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETBRIDGECONFIGRESPONSE")
  (make-package "KORTEX_DRIVER::GETBRIDGECONFIGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetBridgeConfigRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetBridgeConfigRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::BridgeIdentifier :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/BridgeIdentifier _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BridgeIdentifier _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BridgeIdentifier _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetBridgeConfigResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetBridgeConfigResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::BridgeConfig :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/BridgeConfig _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BridgeConfig _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BridgeConfig _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetBridgeConfig
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetBridgeConfig :md5sum-) "9f7aa46e9ca64f4d1fa2fe2894e0afa6")
(setf (get kortex_driver::GetBridgeConfig :datatype-) "kortex_driver/GetBridgeConfig")
(setf (get kortex_driver::GetBridgeConfig :request) kortex_driver::GetBridgeConfigRequest)
(setf (get kortex_driver::GetBridgeConfig :response) kortex_driver::GetBridgeConfigResponse)

(defmethod kortex_driver::GetBridgeConfigRequest
  (:response () (instance kortex_driver::GetBridgeConfigResponse :init)))

(setf (get kortex_driver::GetBridgeConfigRequest :md5sum-) "9f7aa46e9ca64f4d1fa2fe2894e0afa6")
(setf (get kortex_driver::GetBridgeConfigRequest :datatype-) "kortex_driver/GetBridgeConfigRequest")
(setf (get kortex_driver::GetBridgeConfigRequest :definition-)
      "BridgeIdentifier input

================================================================================
MSG: kortex_driver/BridgeIdentifier

---
BridgeConfig output

================================================================================
MSG: kortex_driver/BridgeConfig

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

")

(setf (get kortex_driver::GetBridgeConfigResponse :md5sum-) "9f7aa46e9ca64f4d1fa2fe2894e0afa6")
(setf (get kortex_driver::GetBridgeConfigResponse :datatype-) "kortex_driver/GetBridgeConfigResponse")
(setf (get kortex_driver::GetBridgeConfigResponse :definition-)
      "BridgeIdentifier input

================================================================================
MSG: kortex_driver/BridgeIdentifier

---
BridgeConfig output

================================================================================
MSG: kortex_driver/BridgeConfig

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

")



(provide :kortex_driver/GetBridgeConfig "9f7aa46e9ca64f4d1fa2fe2894e0afa6")


