;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WifiInformationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WifiInformationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WIFIINFORMATIONLIST")
  (make-package "KORTEX_DRIVER::WIFIINFORMATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude WifiInformationList.msg.html


(defclass kortex_driver::WifiInformationList
  :super ros::object
  :slots (_wifi_information_list ))

(defmethod kortex_driver::WifiInformationList
  (:init
   (&key
    ((:wifi_information_list __wifi_information_list) ())
    )
   (send-super :init)
   (setq _wifi_information_list __wifi_information_list)
   self)
  (:wifi_information_list
   (&rest __wifi_information_list)
   (if (keywordp (car __wifi_information_list))
       (send* _wifi_information_list __wifi_information_list)
     (progn
       (if __wifi_information_list (setq _wifi_information_list (car __wifi_information_list)))
       _wifi_information_list)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/WifiInformation[] _wifi_information_list
    (apply #'+ (send-all _wifi_information_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiInformation[] _wifi_information_list
     (write-long (length _wifi_information_list) s)
     (dolist (elem _wifi_information_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiInformation[] _wifi_information_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wifi_information_list (let (r) (dotimes (i n) (push (instance kortex_driver::WifiInformation :init) r)) r))
     (dolist (elem- _wifi_information_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::WifiInformationList :md5sum-) "e35665ac97a2d2d722059d5f54d85ef4")
(setf (get kortex_driver::WifiInformationList :datatype-) "kortex_driver/WifiInformationList")
(setf (get kortex_driver::WifiInformationList :definition-)
      "
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

string identifier
")



(provide :kortex_driver/WifiInformationList "e35665ac97a2d2d722059d5f54d85ef4")


