;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetConfiguredWifi)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetConfiguredWifi (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCONFIGUREDWIFI")
  (make-package "KORTEX_DRIVER::GETCONFIGUREDWIFI"))
(unless (find-package "KORTEX_DRIVER::GETCONFIGUREDWIFIREQUEST")
  (make-package "KORTEX_DRIVER::GETCONFIGUREDWIFIREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCONFIGUREDWIFIRESPONSE")
  (make-package "KORTEX_DRIVER::GETCONFIGUREDWIFIRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetConfiguredWifiRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetConfiguredWifiRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Ssid :init))
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
    ;; kortex_driver/Ssid _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Ssid _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Ssid _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetConfiguredWifiResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetConfiguredWifiResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::WifiConfiguration :init))
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
    ;; kortex_driver/WifiConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetConfiguredWifi
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetConfiguredWifi :md5sum-) "5d1a4bbd14ba02b4b58bf8863a4fba51")
(setf (get kortex_driver::GetConfiguredWifi :datatype-) "kortex_driver/GetConfiguredWifi")
(setf (get kortex_driver::GetConfiguredWifi :request) kortex_driver::GetConfiguredWifiRequest)
(setf (get kortex_driver::GetConfiguredWifi :response) kortex_driver::GetConfiguredWifiResponse)

(defmethod kortex_driver::GetConfiguredWifiRequest
  (:response () (instance kortex_driver::GetConfiguredWifiResponse :init)))

(setf (get kortex_driver::GetConfiguredWifiRequest :md5sum-) "5d1a4bbd14ba02b4b58bf8863a4fba51")
(setf (get kortex_driver::GetConfiguredWifiRequest :datatype-) "kortex_driver/GetConfiguredWifiRequest")
(setf (get kortex_driver::GetConfiguredWifiRequest :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

---
WifiConfiguration output

================================================================================
MSG: kortex_driver/WifiConfiguration

Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

")

(setf (get kortex_driver::GetConfiguredWifiResponse :md5sum-) "5d1a4bbd14ba02b4b58bf8863a4fba51")
(setf (get kortex_driver::GetConfiguredWifiResponse :datatype-) "kortex_driver/GetConfiguredWifiResponse")
(setf (get kortex_driver::GetConfiguredWifiResponse :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

---
WifiConfiguration output

================================================================================
MSG: kortex_driver/WifiConfiguration

Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

")



(provide :kortex_driver/GetConfiguredWifi "5d1a4bbd14ba02b4b58bf8863a4fba51")


