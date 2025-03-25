;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::IsCommunicationInterfaceEnable)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'IsCommunicationInterfaceEnable (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ISCOMMUNICATIONINTERFACEENABLE")
  (make-package "KORTEX_DRIVER::ISCOMMUNICATIONINTERFACEENABLE"))
(unless (find-package "KORTEX_DRIVER::ISCOMMUNICATIONINTERFACEENABLEREQUEST")
  (make-package "KORTEX_DRIVER::ISCOMMUNICATIONINTERFACEENABLEREQUEST"))
(unless (find-package "KORTEX_DRIVER::ISCOMMUNICATIONINTERFACEENABLERESPONSE")
  (make-package "KORTEX_DRIVER::ISCOMMUNICATIONINTERFACEENABLERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::IsCommunicationInterfaceEnableRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::IsCommunicationInterfaceEnableRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::NetworkHandle :init))
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
    ;; kortex_driver/NetworkHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/NetworkHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/NetworkHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::IsCommunicationInterfaceEnableResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::IsCommunicationInterfaceEnableResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CommunicationInterfaceConfiguration :init))
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
    ;; kortex_driver/CommunicationInterfaceConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CommunicationInterfaceConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CommunicationInterfaceConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::IsCommunicationInterfaceEnable
  :super ros::object
  :slots ())

(setf (get kortex_driver::IsCommunicationInterfaceEnable :md5sum-) "7b7c3f92182fedb31e77cfcc39090ac1")
(setf (get kortex_driver::IsCommunicationInterfaceEnable :datatype-) "kortex_driver/IsCommunicationInterfaceEnable")
(setf (get kortex_driver::IsCommunicationInterfaceEnable :request) kortex_driver::IsCommunicationInterfaceEnableRequest)
(setf (get kortex_driver::IsCommunicationInterfaceEnable :response) kortex_driver::IsCommunicationInterfaceEnableResponse)

(defmethod kortex_driver::IsCommunicationInterfaceEnableRequest
  (:response () (instance kortex_driver::IsCommunicationInterfaceEnableResponse :init)))

(setf (get kortex_driver::IsCommunicationInterfaceEnableRequest :md5sum-) "7b7c3f92182fedb31e77cfcc39090ac1")
(setf (get kortex_driver::IsCommunicationInterfaceEnableRequest :datatype-) "kortex_driver/IsCommunicationInterfaceEnableRequest")
(setf (get kortex_driver::IsCommunicationInterfaceEnableRequest :definition-)
      "NetworkHandle input

================================================================================
MSG: kortex_driver/NetworkHandle

---
CommunicationInterfaceConfiguration output

================================================================================
MSG: kortex_driver/CommunicationInterfaceConfiguration

uint32 type
")

(setf (get kortex_driver::IsCommunicationInterfaceEnableResponse :md5sum-) "7b7c3f92182fedb31e77cfcc39090ac1")
(setf (get kortex_driver::IsCommunicationInterfaceEnableResponse :datatype-) "kortex_driver/IsCommunicationInterfaceEnableResponse")
(setf (get kortex_driver::IsCommunicationInterfaceEnableResponse :definition-)
      "NetworkHandle input

================================================================================
MSG: kortex_driver/NetworkHandle

---
CommunicationInterfaceConfiguration output

================================================================================
MSG: kortex_driver/CommunicationInterfaceConfiguration

uint32 type
")



(provide :kortex_driver/IsCommunicationInterfaceEnable "7b7c3f92182fedb31e77cfcc39090ac1")


