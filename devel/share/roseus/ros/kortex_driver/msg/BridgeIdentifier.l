;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BridgeIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BridgeIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRIDGEIDENTIFIER")
  (make-package "KORTEX_DRIVER::BRIDGEIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude BridgeIdentifier.msg.html


(defclass kortex_driver::BridgeIdentifier
  :super ros::object
  :slots (_bridge_id ))

(defmethod kortex_driver::BridgeIdentifier
  (:init
   (&key
    ((:bridge_id __bridge_id) 0)
    )
   (send-super :init)
   (setq _bridge_id (round __bridge_id))
   self)
  (:bridge_id
   (&optional __bridge_id)
   (if __bridge_id (setq _bridge_id __bridge_id)) _bridge_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _bridge_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _bridge_id
       (write-long _bridge_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _bridge_id
     (setq _bridge_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::BridgeIdentifier :md5sum-) "24fa834626ec2b7a0c103b00131d6015")
(setf (get kortex_driver::BridgeIdentifier :datatype-) "kortex_driver/BridgeIdentifier")
(setf (get kortex_driver::BridgeIdentifier :definition-)
      "
uint32 bridge_id
")



(provide :kortex_driver/BridgeIdentifier "24fa834626ec2b7a0c103b00131d6015")


