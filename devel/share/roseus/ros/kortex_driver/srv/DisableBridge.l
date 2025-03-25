;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DisableBridge)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DisableBridge (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DISABLEBRIDGE")
  (make-package "KORTEX_DRIVER::DISABLEBRIDGE"))
(unless (find-package "KORTEX_DRIVER::DISABLEBRIDGEREQUEST")
  (make-package "KORTEX_DRIVER::DISABLEBRIDGEREQUEST"))
(unless (find-package "KORTEX_DRIVER::DISABLEBRIDGERESPONSE")
  (make-package "KORTEX_DRIVER::DISABLEBRIDGERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DisableBridgeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DisableBridgeRequest
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

(defclass kortex_driver::DisableBridgeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DisableBridgeResponse
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

(defclass kortex_driver::DisableBridge
  :super ros::object
  :slots ())

(setf (get kortex_driver::DisableBridge :md5sum-) "5d03590e711a693b2b6ee914ec9cc8b3")
(setf (get kortex_driver::DisableBridge :datatype-) "kortex_driver/DisableBridge")
(setf (get kortex_driver::DisableBridge :request) kortex_driver::DisableBridgeRequest)
(setf (get kortex_driver::DisableBridge :response) kortex_driver::DisableBridgeResponse)

(defmethod kortex_driver::DisableBridgeRequest
  (:response () (instance kortex_driver::DisableBridgeResponse :init)))

(setf (get kortex_driver::DisableBridgeRequest :md5sum-) "5d03590e711a693b2b6ee914ec9cc8b3")
(setf (get kortex_driver::DisableBridgeRequest :datatype-) "kortex_driver/DisableBridgeRequest")
(setf (get kortex_driver::DisableBridgeRequest :definition-)
      "BridgeIdentifier input

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

(setf (get kortex_driver::DisableBridgeResponse :md5sum-) "5d03590e711a693b2b6ee914ec9cc8b3")
(setf (get kortex_driver::DisableBridgeResponse :datatype-) "kortex_driver/DisableBridgeResponse")
(setf (get kortex_driver::DisableBridgeResponse :definition-)
      "BridgeIdentifier input

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



(provide :kortex_driver/DisableBridge "5d03590e711a693b2b6ee914ec9cc8b3")


