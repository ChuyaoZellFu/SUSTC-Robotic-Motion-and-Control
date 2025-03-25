;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BridgeList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BridgeList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRIDGELIST")
  (make-package "KORTEX_DRIVER::BRIDGELIST"))

(in-package "ROS")
;;//! \htmlinclude BridgeList.msg.html


(defclass kortex_driver::BridgeList
  :super ros::object
  :slots (_bridgeConfig ))

(defmethod kortex_driver::BridgeList
  (:init
   (&key
    ((:bridgeConfig __bridgeConfig) ())
    )
   (send-super :init)
   (setq _bridgeConfig __bridgeConfig)
   self)
  (:bridgeConfig
   (&rest __bridgeConfig)
   (if (keywordp (car __bridgeConfig))
       (send* _bridgeConfig __bridgeConfig)
     (progn
       (if __bridgeConfig (setq _bridgeConfig (car __bridgeConfig)))
       _bridgeConfig)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/BridgeConfig[] _bridgeConfig
    (apply #'+ (send-all _bridgeConfig :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BridgeConfig[] _bridgeConfig
     (write-long (length _bridgeConfig) s)
     (dolist (elem _bridgeConfig)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BridgeConfig[] _bridgeConfig
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bridgeConfig (let (r) (dotimes (i n) (push (instance kortex_driver::BridgeConfig :init) r)) r))
     (dolist (elem- _bridgeConfig)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::BridgeList :md5sum-) "67edf56b8cf474f747335e24413448f0")
(setf (get kortex_driver::BridgeList :datatype-) "kortex_driver/BridgeList")
(setf (get kortex_driver::BridgeList :definition-)
      "
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

uint32 bridge_id
")



(provide :kortex_driver/BridgeList "67edf56b8cf474f747335e24413448f0")


