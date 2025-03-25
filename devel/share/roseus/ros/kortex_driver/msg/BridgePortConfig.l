;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BridgePortConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BridgePortConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BRIDGEPORTCONFIG")
  (make-package "KORTEX_DRIVER::BRIDGEPORTCONFIG"))

(in-package "ROS")
;;//! \htmlinclude BridgePortConfig.msg.html


(defclass kortex_driver::BridgePortConfig
  :super ros::object
  :slots (_target_port _out_port ))

(defmethod kortex_driver::BridgePortConfig
  (:init
   (&key
    ((:target_port __target_port) 0)
    ((:out_port __out_port) 0)
    )
   (send-super :init)
   (setq _target_port (round __target_port))
   (setq _out_port (round __out_port))
   self)
  (:target_port
   (&optional __target_port)
   (if __target_port (setq _target_port __target_port)) _target_port)
  (:out_port
   (&optional __out_port)
   (if __out_port (setq _out_port __out_port)) _out_port)
  (:serialization-length
   ()
   (+
    ;; uint32 _target_port
    4
    ;; uint32 _out_port
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _target_port
       (write-long _target_port s)
     ;; uint32 _out_port
       (write-long _out_port s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _target_port
     (setq _target_port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _out_port
     (setq _out_port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::BridgePortConfig :md5sum-) "c2591c5a923ec65c5c9c9b1eb43de273")
(setf (get kortex_driver::BridgePortConfig :datatype-) "kortex_driver/BridgePortConfig")
(setf (get kortex_driver::BridgePortConfig :definition-)
      "
uint32 target_port
uint32 out_port
")



(provide :kortex_driver/BridgePortConfig "c2591c5a923ec65c5c9c9b1eb43de273")


