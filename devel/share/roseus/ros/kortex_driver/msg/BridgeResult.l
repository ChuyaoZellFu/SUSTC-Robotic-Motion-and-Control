;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BridgeResult)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BridgeResult (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRIDGERESULT")
  (make-package "KORTEX_DRIVER::BRIDGERESULT"))

(in-package "ROS")
;;//! \htmlinclude BridgeResult.msg.html


(defclass kortex_driver::BridgeResult
  :super ros::object
  :slots (_bridge_id _status ))

(defmethod kortex_driver::BridgeResult
  (:init
   (&key
    ((:bridge_id __bridge_id) (instance kortex_driver::BridgeIdentifier :init))
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _bridge_id __bridge_id)
   (setq _status (round __status))
   self)
  (:bridge_id
   (&rest __bridge_id)
   (if (keywordp (car __bridge_id))
       (send* _bridge_id __bridge_id)
     (progn
       (if __bridge_id (setq _bridge_id (car __bridge_id)))
       _bridge_id)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/BridgeIdentifier _bridge_id
    (send _bridge_id :serialization-length)
    ;; uint32 _status
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/BridgeIdentifier _bridge_id
       (send _bridge_id :serialize s)
     ;; uint32 _status
       (write-long _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/BridgeIdentifier _bridge_id
     (send _bridge_id :deserialize buf ptr-) (incf ptr- (send _bridge_id :serialization-length))
   ;; uint32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::BridgeResult :md5sum-) "c86e99efcef2f596ae80430e65714cbc")
(setf (get kortex_driver::BridgeResult :datatype-) "kortex_driver/BridgeResult")
(setf (get kortex_driver::BridgeResult :definition-)
      "
BridgeIdentifier bridge_id
uint32 status
================================================================================
MSG: kortex_driver/BridgeIdentifier

uint32 bridge_id
")



(provide :kortex_driver/BridgeResult "c86e99efcef2f596ae80430e65714cbc")


