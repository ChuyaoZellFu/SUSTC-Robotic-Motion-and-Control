;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetCommunicationInterfaceEnable)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetCommunicationInterfaceEnable (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCOMMUNICATIONINTERFACEENABLE")
  (make-package "KORTEX_DRIVER::SETCOMMUNICATIONINTERFACEENABLE"))
(unless (find-package "KORTEX_DRIVER::SETCOMMUNICATIONINTERFACEENABLEREQUEST")
  (make-package "KORTEX_DRIVER::SETCOMMUNICATIONINTERFACEENABLEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCOMMUNICATIONINTERFACEENABLERESPONSE")
  (make-package "KORTEX_DRIVER::SETCOMMUNICATIONINTERFACEENABLERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetCommunicationInterfaceEnableRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetCommunicationInterfaceEnableRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::CommunicationInterfaceConfiguration :init))
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
    ;; kortex_driver/CommunicationInterfaceConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CommunicationInterfaceConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CommunicationInterfaceConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetCommunicationInterfaceEnableResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetCommunicationInterfaceEnableResponse
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

(defclass kortex_driver::SetCommunicationInterfaceEnable
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetCommunicationInterfaceEnable :md5sum-) "ea61eebb1ea7afcd99f7fa2e0cb8d9db")
(setf (get kortex_driver::SetCommunicationInterfaceEnable :datatype-) "kortex_driver/SetCommunicationInterfaceEnable")
(setf (get kortex_driver::SetCommunicationInterfaceEnable :request) kortex_driver::SetCommunicationInterfaceEnableRequest)
(setf (get kortex_driver::SetCommunicationInterfaceEnable :response) kortex_driver::SetCommunicationInterfaceEnableResponse)

(defmethod kortex_driver::SetCommunicationInterfaceEnableRequest
  (:response () (instance kortex_driver::SetCommunicationInterfaceEnableResponse :init)))

(setf (get kortex_driver::SetCommunicationInterfaceEnableRequest :md5sum-) "ea61eebb1ea7afcd99f7fa2e0cb8d9db")
(setf (get kortex_driver::SetCommunicationInterfaceEnableRequest :datatype-) "kortex_driver/SetCommunicationInterfaceEnableRequest")
(setf (get kortex_driver::SetCommunicationInterfaceEnableRequest :definition-)
      "CommunicationInterfaceConfiguration input

================================================================================
MSG: kortex_driver/CommunicationInterfaceConfiguration

uint32 type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetCommunicationInterfaceEnableResponse :md5sum-) "ea61eebb1ea7afcd99f7fa2e0cb8d9db")
(setf (get kortex_driver::SetCommunicationInterfaceEnableResponse :datatype-) "kortex_driver/SetCommunicationInterfaceEnableResponse")
(setf (get kortex_driver::SetCommunicationInterfaceEnableResponse :definition-)
      "CommunicationInterfaceConfiguration input

================================================================================
MSG: kortex_driver/CommunicationInterfaceConfiguration

uint32 type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetCommunicationInterfaceEnable "ea61eebb1ea7afcd99f7fa2e0cb8d9db")


