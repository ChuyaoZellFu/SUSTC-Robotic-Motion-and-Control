;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EnableBridge)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EnableBridge (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ENABLEBRIDGE")
  (make-package "KORTEX_DRIVER::ENABLEBRIDGE"))
(unless (find-package "KORTEX_DRIVER::ENABLEBRIDGEREQUEST")
  (make-package "KORTEX_DRIVER::ENABLEBRIDGEREQUEST"))
(unless (find-package "KORTEX_DRIVER::ENABLEBRIDGERESPONSE")
  (make-package "KORTEX_DRIVER::ENABLEBRIDGERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::EnableBridgeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::EnableBridgeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::BridgeConfig :init))
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
    ;; kortex_driver/BridgeConfig _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BridgeConfig _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BridgeConfig _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::EnableBridgeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::EnableBridgeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::BridgeResult :init))
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
    ;; kortex_driver/BridgeResult _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BridgeResult _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BridgeResult _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::EnableBridge
  :super ros::object
  :slots ())

(setf (get kortex_driver::EnableBridge :md5sum-) "be9e21afaa7e92c6bf00b8eed13a695e")
(setf (get kortex_driver::EnableBridge :datatype-) "kortex_driver/EnableBridge")
(setf (get kortex_driver::EnableBridge :request) kortex_driver::EnableBridgeRequest)
(setf (get kortex_driver::EnableBridge :response) kortex_driver::EnableBridgeResponse)

(defmethod kortex_driver::EnableBridgeRequest
  (:response () (instance kortex_driver::EnableBridgeResponse :init)))

(setf (get kortex_driver::EnableBridgeRequest :md5sum-) "be9e21afaa7e92c6bf00b8eed13a695e")
(setf (get kortex_driver::EnableBridgeRequest :datatype-) "kortex_driver/EnableBridgeRequest")
(setf (get kortex_driver::EnableBridgeRequest :definition-)
      "BridgeConfig input

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

---
BridgeResult output

================================================================================
MSG: kortex_driver/BridgeResult

BridgeIdentifier bridge_id
uint32 status
================================================================================
MSG: kortex_driver/BridgeIdentifier

")

(setf (get kortex_driver::EnableBridgeResponse :md5sum-) "be9e21afaa7e92c6bf00b8eed13a695e")
(setf (get kortex_driver::EnableBridgeResponse :datatype-) "kortex_driver/EnableBridgeResponse")
(setf (get kortex_driver::EnableBridgeResponse :definition-)
      "BridgeConfig input

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

---
BridgeResult output

================================================================================
MSG: kortex_driver/BridgeResult

BridgeIdentifier bridge_id
uint32 status
================================================================================
MSG: kortex_driver/BridgeIdentifier

")



(provide :kortex_driver/EnableBridge "be9e21afaa7e92c6bf00b8eed13a695e")


