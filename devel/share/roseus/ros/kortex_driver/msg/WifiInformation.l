;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WifiInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WifiInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WIFIINFORMATION")
  (make-package "KORTEX_DRIVER::WIFIINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude WifiInformation.msg.html


(defclass kortex_driver::WifiInformation
  :super ros::object
  :slots (_ssid _security_type _encryption_type _signal_quality _signal_strength _frequency _channel ))

(defmethod kortex_driver::WifiInformation
  (:init
   (&key
    ((:ssid __ssid) (instance kortex_driver::Ssid :init))
    ((:security_type __security_type) 0)
    ((:encryption_type __encryption_type) 0)
    ((:signal_quality __signal_quality) 0)
    ((:signal_strength __signal_strength) 0)
    ((:frequency __frequency) 0)
    ((:channel __channel) 0)
    )
   (send-super :init)
   (setq _ssid __ssid)
   (setq _security_type (round __security_type))
   (setq _encryption_type (round __encryption_type))
   (setq _signal_quality (round __signal_quality))
   (setq _signal_strength (round __signal_strength))
   (setq _frequency (round __frequency))
   (setq _channel (round __channel))
   self)
  (:ssid
   (&rest __ssid)
   (if (keywordp (car __ssid))
       (send* _ssid __ssid)
     (progn
       (if __ssid (setq _ssid (car __ssid)))
       _ssid)))
  (:security_type
   (&optional __security_type)
   (if __security_type (setq _security_type __security_type)) _security_type)
  (:encryption_type
   (&optional __encryption_type)
   (if __encryption_type (setq _encryption_type __encryption_type)) _encryption_type)
  (:signal_quality
   (&optional __signal_quality)
   (if __signal_quality (setq _signal_quality __signal_quality)) _signal_quality)
  (:signal_strength
   (&optional __signal_strength)
   (if __signal_strength (setq _signal_strength __signal_strength)) _signal_strength)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Ssid _ssid
    (send _ssid :serialization-length)
    ;; uint32 _security_type
    4
    ;; uint32 _encryption_type
    4
    ;; uint32 _signal_quality
    4
    ;; int32 _signal_strength
    4
    ;; uint32 _frequency
    4
    ;; uint32 _channel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Ssid _ssid
       (send _ssid :serialize s)
     ;; uint32 _security_type
       (write-long _security_type s)
     ;; uint32 _encryption_type
       (write-long _encryption_type s)
     ;; uint32 _signal_quality
       (write-long _signal_quality s)
     ;; int32 _signal_strength
       (write-long _signal_strength s)
     ;; uint32 _frequency
       (write-long _frequency s)
     ;; uint32 _channel
       (write-long _channel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Ssid _ssid
     (send _ssid :deserialize buf ptr-) (incf ptr- (send _ssid :serialization-length))
   ;; uint32 _security_type
     (setq _security_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _encryption_type
     (setq _encryption_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _signal_quality
     (setq _signal_quality (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _signal_strength
     (setq _signal_strength (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _frequency
     (setq _frequency (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _channel
     (setq _channel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::WifiInformation :md5sum-) "6cd167c73dec6e8468316540111c37c6")
(setf (get kortex_driver::WifiInformation :datatype-) "kortex_driver/WifiInformation")
(setf (get kortex_driver::WifiInformation :definition-)
      "
Ssid ssid
uint32 security_type
uint32 encryption_type
uint32 signal_quality
int32 signal_strength
uint32 frequency
uint32 channel
================================================================================
MSG: kortex_driver/Ssid

string identifier
")



(provide :kortex_driver/WifiInformation "6cd167c73dec6e8468316540111c37c6")


