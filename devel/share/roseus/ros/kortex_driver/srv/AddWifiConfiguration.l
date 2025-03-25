;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::AddWifiConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'AddWifiConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ADDWIFICONFIGURATION")
  (make-package "KORTEX_DRIVER::ADDWIFICONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::ADDWIFICONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::ADDWIFICONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::ADDWIFICONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::ADDWIFICONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::AddWifiConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::AddWifiConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::WifiConfiguration :init))
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
    ;; kortex_driver/WifiConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::AddWifiConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::AddWifiConfigurationResponse
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

(defclass kortex_driver::AddWifiConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::AddWifiConfiguration :md5sum-) "2cc46423b77629bdba4927dbabe3a2cb")
(setf (get kortex_driver::AddWifiConfiguration :datatype-) "kortex_driver/AddWifiConfiguration")
(setf (get kortex_driver::AddWifiConfiguration :request) kortex_driver::AddWifiConfigurationRequest)
(setf (get kortex_driver::AddWifiConfiguration :response) kortex_driver::AddWifiConfigurationResponse)

(defmethod kortex_driver::AddWifiConfigurationRequest
  (:response () (instance kortex_driver::AddWifiConfigurationResponse :init)))

(setf (get kortex_driver::AddWifiConfigurationRequest :md5sum-) "2cc46423b77629bdba4927dbabe3a2cb")
(setf (get kortex_driver::AddWifiConfigurationRequest :datatype-) "kortex_driver/AddWifiConfigurationRequest")
(setf (get kortex_driver::AddWifiConfigurationRequest :definition-)
      "WifiConfiguration input

================================================================================
MSG: kortex_driver/WifiConfiguration

Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::AddWifiConfigurationResponse :md5sum-) "2cc46423b77629bdba4927dbabe3a2cb")
(setf (get kortex_driver::AddWifiConfigurationResponse :datatype-) "kortex_driver/AddWifiConfigurationResponse")
(setf (get kortex_driver::AddWifiConfigurationResponse :definition-)
      "WifiConfiguration input

================================================================================
MSG: kortex_driver/WifiConfiguration

Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/AddWifiConfiguration "2cc46423b77629bdba4927dbabe3a2cb")


