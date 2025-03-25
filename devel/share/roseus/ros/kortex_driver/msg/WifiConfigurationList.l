;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WifiConfigurationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WifiConfigurationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WIFICONFIGURATIONLIST")
  (make-package "KORTEX_DRIVER::WIFICONFIGURATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude WifiConfigurationList.msg.html


(defclass kortex_driver::WifiConfigurationList
  :super ros::object
  :slots (_wifi_configuration_list ))

(defmethod kortex_driver::WifiConfigurationList
  (:init
   (&key
    ((:wifi_configuration_list __wifi_configuration_list) ())
    )
   (send-super :init)
   (setq _wifi_configuration_list __wifi_configuration_list)
   self)
  (:wifi_configuration_list
   (&rest __wifi_configuration_list)
   (if (keywordp (car __wifi_configuration_list))
       (send* _wifi_configuration_list __wifi_configuration_list)
     (progn
       (if __wifi_configuration_list (setq _wifi_configuration_list (car __wifi_configuration_list)))
       _wifi_configuration_list)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/WifiConfiguration[] _wifi_configuration_list
    (apply #'+ (send-all _wifi_configuration_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiConfiguration[] _wifi_configuration_list
     (write-long (length _wifi_configuration_list) s)
     (dolist (elem _wifi_configuration_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiConfiguration[] _wifi_configuration_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wifi_configuration_list (let (r) (dotimes (i n) (push (instance kortex_driver::WifiConfiguration :init) r)) r))
     (dolist (elem- _wifi_configuration_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::WifiConfigurationList :md5sum-) "826e37c04fad51b5554819fbd5e69cc8")
(setf (get kortex_driver::WifiConfigurationList :datatype-) "kortex_driver/WifiConfigurationList")
(setf (get kortex_driver::WifiConfigurationList :definition-)
      "
WifiConfiguration[] wifi_configuration_list
================================================================================
MSG: kortex_driver/WifiConfiguration

Ssid ssid
string security_key
bool connect_automatically
================================================================================
MSG: kortex_driver/Ssid

string identifier
")



(provide :kortex_driver/WifiConfigurationList "826e37c04fad51b5554819fbd5e69cc8")


