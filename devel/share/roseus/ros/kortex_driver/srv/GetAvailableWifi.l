;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAvailableWifi)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAvailableWifi (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETAVAILABLEWIFI")
  (make-package "KORTEX_DRIVER::GETAVAILABLEWIFI"))
(unless (find-package "KORTEX_DRIVER::GETAVAILABLEWIFIREQUEST")
  (make-package "KORTEX_DRIVER::GETAVAILABLEWIFIREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETAVAILABLEWIFIRESPONSE")
  (make-package "KORTEX_DRIVER::GETAVAILABLEWIFIRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAvailableWifiRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAvailableWifiRequest
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

(defclass kortex_driver::GetAvailableWifiResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAvailableWifiResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::WifiInformationList :init))
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
    ;; kortex_driver/WifiInformationList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiInformationList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiInformationList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAvailableWifi
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAvailableWifi :md5sum-) "7759436e458092e2d8f72d35d5d731f7")
(setf (get kortex_driver::GetAvailableWifi :datatype-) "kortex_driver/GetAvailableWifi")
(setf (get kortex_driver::GetAvailableWifi :request) kortex_driver::GetAvailableWifiRequest)
(setf (get kortex_driver::GetAvailableWifi :response) kortex_driver::GetAvailableWifiResponse)

(defmethod kortex_driver::GetAvailableWifiRequest
  (:response () (instance kortex_driver::GetAvailableWifiResponse :init)))

(setf (get kortex_driver::GetAvailableWifiRequest :md5sum-) "7759436e458092e2d8f72d35d5d731f7")
(setf (get kortex_driver::GetAvailableWifiRequest :datatype-) "kortex_driver/GetAvailableWifiRequest")
(setf (get kortex_driver::GetAvailableWifiRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiInformationList output

================================================================================
MSG: kortex_driver/WifiInformationList

WifiInformation[] wifi_information_list
================================================================================
MSG: kortex_driver/WifiInformation

Ssid ssid
uint32 security_type
uint32 encryption_type
uint32 signal_quality
int32 signal_strength
uint32 frequency
uint32 channel
================================================================================
MSG: kortex_driver/Ssid

")

(setf (get kortex_driver::GetAvailableWifiResponse :md5sum-) "7759436e458092e2d8f72d35d5d731f7")
(setf (get kortex_driver::GetAvailableWifiResponse :datatype-) "kortex_driver/GetAvailableWifiResponse")
(setf (get kortex_driver::GetAvailableWifiResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiInformationList output

================================================================================
MSG: kortex_driver/WifiInformationList

WifiInformation[] wifi_information_list
================================================================================
MSG: kortex_driver/WifiInformation

Ssid ssid
uint32 security_type
uint32 encryption_type
uint32 signal_quality
int32 signal_strength
uint32 frequency
uint32 channel
================================================================================
MSG: kortex_driver/Ssid

")



(provide :kortex_driver/GetAvailableWifi "7759436e458092e2d8f72d35d5d731f7")


