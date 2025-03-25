;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetIPv4Configuration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetIPv4Configuration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETIPV4CONFIGURATION")
  (make-package "KORTEX_DRIVER::SETIPV4CONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETIPV4CONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETIPV4CONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETIPV4CONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETIPV4CONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetIPv4ConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetIPv4ConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::FullIPv4Configuration :init))
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
    ;; kortex_driver/FullIPv4Configuration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/FullIPv4Configuration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/FullIPv4Configuration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetIPv4ConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetIPv4ConfigurationResponse
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

(defclass kortex_driver::SetIPv4Configuration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetIPv4Configuration :md5sum-) "efb45709f10aca05c41eab84d8334547")
(setf (get kortex_driver::SetIPv4Configuration :datatype-) "kortex_driver/SetIPv4Configuration")
(setf (get kortex_driver::SetIPv4Configuration :request) kortex_driver::SetIPv4ConfigurationRequest)
(setf (get kortex_driver::SetIPv4Configuration :response) kortex_driver::SetIPv4ConfigurationResponse)

(defmethod kortex_driver::SetIPv4ConfigurationRequest
  (:response () (instance kortex_driver::SetIPv4ConfigurationResponse :init)))

(setf (get kortex_driver::SetIPv4ConfigurationRequest :md5sum-) "efb45709f10aca05c41eab84d8334547")
(setf (get kortex_driver::SetIPv4ConfigurationRequest :datatype-) "kortex_driver/SetIPv4ConfigurationRequest")
(setf (get kortex_driver::SetIPv4ConfigurationRequest :definition-)
      "FullIPv4Configuration input

================================================================================
MSG: kortex_driver/FullIPv4Configuration

NetworkHandle handle
IPv4Configuration ipv4_configuration
================================================================================
MSG: kortex_driver/NetworkHandle

uint32 type
================================================================================
MSG: kortex_driver/IPv4Configuration

uint32 ip_address
uint32 subnet_mask
uint32 default_gateway
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetIPv4ConfigurationResponse :md5sum-) "efb45709f10aca05c41eab84d8334547")
(setf (get kortex_driver::SetIPv4ConfigurationResponse :datatype-) "kortex_driver/SetIPv4ConfigurationResponse")
(setf (get kortex_driver::SetIPv4ConfigurationResponse :definition-)
      "FullIPv4Configuration input

================================================================================
MSG: kortex_driver/FullIPv4Configuration

NetworkHandle handle
IPv4Configuration ipv4_configuration
================================================================================
MSG: kortex_driver/NetworkHandle

uint32 type
================================================================================
MSG: kortex_driver/IPv4Configuration

uint32 ip_address
uint32 subnet_mask
uint32 default_gateway
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetIPv4Configuration "efb45709f10aca05c41eab84d8334547")


