;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAllConfiguredWifis)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAllConfiguredWifis (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETALLCONFIGUREDWIFIS")
  (make-package "KORTEX_DRIVER::GETALLCONFIGUREDWIFIS"))
(unless (find-package "KORTEX_DRIVER::GETALLCONFIGUREDWIFISREQUEST")
  (make-package "KORTEX_DRIVER::GETALLCONFIGUREDWIFISREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETALLCONFIGUREDWIFISRESPONSE")
  (make-package "KORTEX_DRIVER::GETALLCONFIGUREDWIFISRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAllConfiguredWifisRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAllConfiguredWifisRequest
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

(defclass kortex_driver::GetAllConfiguredWifisResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAllConfiguredWifisResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::WifiConfigurationList :init))
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
    ;; kortex_driver/WifiConfigurationList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiConfigurationList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiConfigurationList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAllConfiguredWifis
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAllConfiguredWifis :md5sum-) "00cc4429e5b474aecc287723cc4eff69")
(setf (get kortex_driver::GetAllConfiguredWifis :datatype-) "kortex_driver/GetAllConfiguredWifis")
(setf (get kortex_driver::GetAllConfiguredWifis :request) kortex_driver::GetAllConfiguredWifisRequest)
(setf (get kortex_driver::GetAllConfiguredWifis :response) kortex_driver::GetAllConfiguredWifisResponse)

(defmethod kortex_driver::GetAllConfiguredWifisRequest
  (:response () (instance kortex_driver::GetAllConfiguredWifisResponse :init)))

(setf (get kortex_driver::GetAllConfiguredWifisRequest :md5sum-) "00cc4429e5b474aecc287723cc4eff69")
(setf (get kortex_driver::GetAllConfiguredWifisRequest :datatype-) "kortex_driver/GetAllConfiguredWifisRequest")
(setf (get kortex_driver::GetAllConfiguredWifisRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiConfigurationList output

================================================================================
MSG: kortex_driver/WifiConfigurationList

WifiConfiguration[] wifi_configuration_list
================================================================================
MSG: kortex_driver/WifiConfiguration

Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

")

(setf (get kortex_driver::GetAllConfiguredWifisResponse :md5sum-) "00cc4429e5b474aecc287723cc4eff69")
(setf (get kortex_driver::GetAllConfiguredWifisResponse :datatype-) "kortex_driver/GetAllConfiguredWifisResponse")
(setf (get kortex_driver::GetAllConfiguredWifisResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiConfigurationList output

================================================================================
MSG: kortex_driver/WifiConfigurationList

WifiConfiguration[] wifi_configuration_list
================================================================================
MSG: kortex_driver/WifiConfiguration

Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

")



(provide :kortex_driver/GetAllConfiguredWifis "00cc4429e5b474aecc287723cc4eff69")


