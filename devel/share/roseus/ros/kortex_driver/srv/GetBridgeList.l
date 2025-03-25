;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetBridgeList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetBridgeList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETBRIDGELIST")
  (make-package "KORTEX_DRIVER::GETBRIDGELIST"))
(unless (find-package "KORTEX_DRIVER::GETBRIDGELISTREQUEST")
  (make-package "KORTEX_DRIVER::GETBRIDGELISTREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETBRIDGELISTRESPONSE")
  (make-package "KORTEX_DRIVER::GETBRIDGELISTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetBridgeListRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetBridgeListRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetBridgeListResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetBridgeListResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::BridgeList :init))
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
    ;; kortex_driver/BridgeList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BridgeList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BridgeList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetBridgeList
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetBridgeList :md5sum-) "866f5d10e4d60286569bdfe7c65c5078")
(setf (get kortex_driver::GetBridgeList :datatype-) "kortex_driver/GetBridgeList")
(setf (get kortex_driver::GetBridgeList :request) kortex_driver::GetBridgeListRequest)
(setf (get kortex_driver::GetBridgeList :response) kortex_driver::GetBridgeListResponse)

(defmethod kortex_driver::GetBridgeListRequest
  (:response () (instance kortex_driver::GetBridgeListResponse :init)))

(setf (get kortex_driver::GetBridgeListRequest :md5sum-) "866f5d10e4d60286569bdfe7c65c5078")
(setf (get kortex_driver::GetBridgeListRequest :datatype-) "kortex_driver/GetBridgeListRequest")
(setf (get kortex_driver::GetBridgeListRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
BridgeList output

================================================================================
MSG: kortex_driver/BridgeList

BridgeConfig[] bridgeConfig
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

(setf (get kortex_driver::GetBridgeListResponse :md5sum-) "866f5d10e4d60286569bdfe7c65c5078")
(setf (get kortex_driver::GetBridgeListResponse :datatype-) "kortex_driver/GetBridgeListResponse")
(setf (get kortex_driver::GetBridgeListResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
BridgeList output

================================================================================
MSG: kortex_driver/BridgeList

BridgeConfig[] bridgeConfig
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



(provide :kortex_driver/GetBridgeList "866f5d10e4d60286569bdfe7c65c5078")


