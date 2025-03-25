;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WifiConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WifiConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WIFICONFIGURATION")
  (make-package "KORTEX_DRIVER::WIFICONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude WifiConfiguration.msg.html


(defclass kortex_driver::WifiConfiguration
  :super ros::object
  :slots (_ssid _security_key _connect_automatically ))

(defmethod kortex_driver::WifiConfiguration
  (:init
   (&key
    ((:ssid __ssid) (instance kortex_driver::Ssid :init))
    ((:security_key __security_key) "")
    ((:connect_automatically __connect_automatically) nil)
    )
   (send-super :init)
   (setq _ssid __ssid)
   (setq _security_key (string __security_key))
   (setq _connect_automatically __connect_automatically)
   self)
  (:ssid
   (&rest __ssid)
   (if (keywordp (car __ssid))
       (send* _ssid __ssid)
     (progn
       (if __ssid (setq _ssid (car __ssid)))
       _ssid)))
  (:security_key
   (&optional __security_key)
   (if __security_key (setq _security_key __security_key)) _security_key)
  (:connect_automatically
   (&optional (__connect_automatically :null))
   (if (not (eq __connect_automatically :null)) (setq _connect_automatically __connect_automatically)) _connect_automatically)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Ssid _ssid
    (send _ssid :serialization-length)
    ;; string _security_key
    4 (length _security_key)
    ;; bool _connect_automatically
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Ssid _ssid
       (send _ssid :serialize s)
     ;; string _security_key
       (write-long (length _security_key) s) (princ _security_key s)
     ;; bool _connect_automatically
       (if _connect_automatically (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Ssid _ssid
     (send _ssid :deserialize buf ptr-) (incf ptr- (send _ssid :serialization-length))
   ;; string _security_key
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _security_key (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _connect_automatically
     (setq _connect_automatically (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::WifiConfiguration :md5sum-) "4308400b333c62f7c2b092d1ea664706")
(setf (get kortex_driver::WifiConfiguration :datatype-) "kortex_driver/WifiConfiguration")
(setf (get kortex_driver::WifiConfiguration :definition-)
      "
Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

string identifier
")



(provide :kortex_driver/WifiConfiguration "4308400b333c62f7c2b092d1ea664706")


