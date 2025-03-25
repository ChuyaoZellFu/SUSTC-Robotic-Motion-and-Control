;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetWifiInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetWifiInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETWIFIINFORMATION")
  (make-package "KORTEX_DRIVER::GETWIFIINFORMATION"))
(unless (find-package "KORTEX_DRIVER::GETWIFIINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETWIFIINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETWIFIINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETWIFIINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetWifiInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetWifiInformationRequest
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

(defclass kortex_driver::GetWifiInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetWifiInformationResponse
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

(defclass kortex_driver::GetWifiInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetWifiInformation :md5sum-) "54ddc4afd24b3f5a522f79c66970e8fc")
(setf (get kortex_driver::GetWifiInformation :datatype-) "kortex_driver/GetWifiInformation")
(setf (get kortex_driver::GetWifiInformation :request) kortex_driver::GetWifiInformationRequest)
(setf (get kortex_driver::GetWifiInformation :response) kortex_driver::GetWifiInformationResponse)

(defmethod kortex_driver::GetWifiInformationRequest
  (:response () (instance kortex_driver::GetWifiInformationResponse :init)))

(setf (get kortex_driver::GetWifiInformationRequest :md5sum-) "54ddc4afd24b3f5a522f79c66970e8fc")
(setf (get kortex_driver::GetWifiInformationRequest :datatype-) "kortex_driver/GetWifiInformationRequest")
(setf (get kortex_driver::GetWifiInformationRequest :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

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

(setf (get kortex_driver::GetWifiInformationResponse :md5sum-) "54ddc4afd24b3f5a522f79c66970e8fc")
(setf (get kortex_driver::GetWifiInformationResponse :datatype-) "kortex_driver/GetWifiInformationResponse")
(setf (get kortex_driver::GetWifiInformationResponse :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

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



(provide :kortex_driver/GetWifiInformation "54ddc4afd24b3f5a522f79c66970e8fc")


