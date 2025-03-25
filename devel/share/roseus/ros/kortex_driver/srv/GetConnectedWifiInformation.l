;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetConnectedWifiInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetConnectedWifiInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCONNECTEDWIFIINFORMATION")
  (make-package "KORTEX_DRIVER::GETCONNECTEDWIFIINFORMATION"))
(unless (find-package "KORTEX_DRIVER::GETCONNECTEDWIFIINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETCONNECTEDWIFIINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCONNECTEDWIFIINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETCONNECTEDWIFIINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetConnectedWifiInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetConnectedWifiInformationRequest
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

(defclass kortex_driver::GetConnectedWifiInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetConnectedWifiInformationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::WifiInformation :init))
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
    ;; kortex_driver/WifiInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetConnectedWifiInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetConnectedWifiInformation :md5sum-) "38ff400e5249598bc1f7289a3e9733aa")
(setf (get kortex_driver::GetConnectedWifiInformation :datatype-) "kortex_driver/GetConnectedWifiInformation")
(setf (get kortex_driver::GetConnectedWifiInformation :request) kortex_driver::GetConnectedWifiInformationRequest)
(setf (get kortex_driver::GetConnectedWifiInformation :response) kortex_driver::GetConnectedWifiInformationResponse)

(defmethod kortex_driver::GetConnectedWifiInformationRequest
  (:response () (instance kortex_driver::GetConnectedWifiInformationResponse :init)))

(setf (get kortex_driver::GetConnectedWifiInformationRequest :md5sum-) "38ff400e5249598bc1f7289a3e9733aa")
(setf (get kortex_driver::GetConnectedWifiInformationRequest :datatype-) "kortex_driver/GetConnectedWifiInformationRequest")
(setf (get kortex_driver::GetConnectedWifiInformationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiInformation output

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

(setf (get kortex_driver::GetConnectedWifiInformationResponse :md5sum-) "38ff400e5249598bc1f7289a3e9733aa")
(setf (get kortex_driver::GetConnectedWifiInformationResponse :datatype-) "kortex_driver/GetConnectedWifiInformationResponse")
(setf (get kortex_driver::GetConnectedWifiInformationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiInformation output

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



(provide :kortex_driver/GetConnectedWifiInformation "38ff400e5249598bc1f7289a3e9733aa")


