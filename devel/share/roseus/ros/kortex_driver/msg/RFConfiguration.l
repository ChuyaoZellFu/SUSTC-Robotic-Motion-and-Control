;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RFConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RFConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RFCONFIGURATION")
  (make-package "KORTEX_DRIVER::RFCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude RFConfiguration.msg.html


(defclass kortex_driver::RFConfiguration
  :super ros::object
  :slots (_wifi_enable_state _bluetooth_enable_state ))

(defmethod kortex_driver::RFConfiguration
  (:init
   (&key
    ((:wifi_enable_state __wifi_enable_state) (instance kortex_driver::WifiEnableState :init))
    ((:bluetooth_enable_state __bluetooth_enable_state) (instance kortex_driver::BluetoothEnableState :init))
    )
   (send-super :init)
   (setq _wifi_enable_state __wifi_enable_state)
   (setq _bluetooth_enable_state __bluetooth_enable_state)
   self)
  (:wifi_enable_state
   (&rest __wifi_enable_state)
   (if (keywordp (car __wifi_enable_state))
       (send* _wifi_enable_state __wifi_enable_state)
     (progn
       (if __wifi_enable_state (setq _wifi_enable_state (car __wifi_enable_state)))
       _wifi_enable_state)))
  (:bluetooth_enable_state
   (&rest __bluetooth_enable_state)
   (if (keywordp (car __bluetooth_enable_state))
       (send* _bluetooth_enable_state __bluetooth_enable_state)
     (progn
       (if __bluetooth_enable_state (setq _bluetooth_enable_state (car __bluetooth_enable_state)))
       _bluetooth_enable_state)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/WifiEnableState _wifi_enable_state
    (send _wifi_enable_state :serialization-length)
    ;; kortex_driver/BluetoothEnableState _bluetooth_enable_state
    (send _bluetooth_enable_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiEnableState _wifi_enable_state
       (send _wifi_enable_state :serialize s)
     ;; kortex_driver/BluetoothEnableState _bluetooth_enable_state
       (send _bluetooth_enable_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiEnableState _wifi_enable_state
     (send _wifi_enable_state :deserialize buf ptr-) (incf ptr- (send _wifi_enable_state :serialization-length))
   ;; kortex_driver/BluetoothEnableState _bluetooth_enable_state
     (send _bluetooth_enable_state :deserialize buf ptr-) (incf ptr- (send _bluetooth_enable_state :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::RFConfiguration :md5sum-) "b8bfe540c76a207913f7638bb2dbbae6")
(setf (get kortex_driver::RFConfiguration :datatype-) "kortex_driver/RFConfiguration")
(setf (get kortex_driver::RFConfiguration :definition-)
      "
WifiEnableState wifi_enable_state
BluetoothEnableState bluetooth_enable_state
================================================================================
MSG: kortex_driver/WifiEnableState

bool enabled
================================================================================
MSG: kortex_driver/BluetoothEnableState

bool enabled
")



(provide :kortex_driver/RFConfiguration "b8bfe540c76a207913f7638bb2dbbae6")


