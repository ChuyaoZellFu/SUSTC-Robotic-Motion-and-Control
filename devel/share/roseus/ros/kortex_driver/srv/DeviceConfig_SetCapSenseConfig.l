;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeviceConfig_SetCapSenseConfig)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeviceConfig_SetCapSenseConfig (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DEVICECONFIG_SETCAPSENSECONFIG")
  (make-package "KORTEX_DRIVER::DEVICECONFIG_SETCAPSENSECONFIG"))
(unless (find-package "KORTEX_DRIVER::DEVICECONFIG_SETCAPSENSECONFIGREQUEST")
  (make-package "KORTEX_DRIVER::DEVICECONFIG_SETCAPSENSECONFIGREQUEST"))
(unless (find-package "KORTEX_DRIVER::DEVICECONFIG_SETCAPSENSECONFIGRESPONSE")
  (make-package "KORTEX_DRIVER::DEVICECONFIG_SETCAPSENSECONFIGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeviceConfig_SetCapSenseConfigRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeviceConfig_SetCapSenseConfigRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::DeviceConfig_CapSenseConfig :init))
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
    ;; kortex_driver/DeviceConfig_CapSenseConfig _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/DeviceConfig_CapSenseConfig _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/DeviceConfig_CapSenseConfig _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DeviceConfig_SetCapSenseConfigResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeviceConfig_SetCapSenseConfigResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DeviceConfig_SetCapSenseConfig
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeviceConfig_SetCapSenseConfig :md5sum-) "af529d9737e7a99888f758db0a779b87")
(setf (get kortex_driver::DeviceConfig_SetCapSenseConfig :datatype-) "kortex_driver/DeviceConfig_SetCapSenseConfig")
(setf (get kortex_driver::DeviceConfig_SetCapSenseConfig :request) kortex_driver::DeviceConfig_SetCapSenseConfigRequest)
(setf (get kortex_driver::DeviceConfig_SetCapSenseConfig :response) kortex_driver::DeviceConfig_SetCapSenseConfigResponse)

(defmethod kortex_driver::DeviceConfig_SetCapSenseConfigRequest
  (:response () (instance kortex_driver::DeviceConfig_SetCapSenseConfigResponse :init)))

(setf (get kortex_driver::DeviceConfig_SetCapSenseConfigRequest :md5sum-) "af529d9737e7a99888f758db0a779b87")
(setf (get kortex_driver::DeviceConfig_SetCapSenseConfigRequest :datatype-) "kortex_driver/DeviceConfig_SetCapSenseConfigRequest")
(setf (get kortex_driver::DeviceConfig_SetCapSenseConfigRequest :definition-)
      "DeviceConfig_CapSenseConfig input

================================================================================
MSG: kortex_driver/DeviceConfig_CapSenseConfig

uint32 mode
float32 threshold_a
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeviceConfig_SetCapSenseConfigResponse :md5sum-) "af529d9737e7a99888f758db0a779b87")
(setf (get kortex_driver::DeviceConfig_SetCapSenseConfigResponse :datatype-) "kortex_driver/DeviceConfig_SetCapSenseConfigResponse")
(setf (get kortex_driver::DeviceConfig_SetCapSenseConfigResponse :definition-)
      "DeviceConfig_CapSenseConfig input

================================================================================
MSG: kortex_driver/DeviceConfig_CapSenseConfig

uint32 mode
float32 threshold_a
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeviceConfig_SetCapSenseConfig "af529d9737e7a99888f758db0a779b87")


