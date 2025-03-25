;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OperatingModeNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OperatingModeNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPERATINGMODENOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::OPERATINGMODENOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude OperatingModeNotificationList.msg.html


(defclass kortex_driver::OperatingModeNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::OperatingModeNotificationList
  (:init
   (&key
    ((:notifications __notifications) ())
    )
   (send-super :init)
   (setq _notifications __notifications)
   self)
  (:notifications
   (&rest __notifications)
   (if (keywordp (car __notifications))
       (send* _notifications __notifications)
     (progn
       (if __notifications (setq _notifications (car __notifications)))
       _notifications)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/OperatingModeNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/OperatingModeNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/OperatingModeNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::OperatingModeNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::OperatingModeNotificationList :md5sum-) "52f234be703b9c3a9743364c4bec1c58")
(setf (get kortex_driver::OperatingModeNotificationList :datatype-) "kortex_driver/OperatingModeNotificationList")
(setf (get kortex_driver::OperatingModeNotificationList :definition-)
      "
OperatingModeNotification[] notifications
================================================================================
MSG: kortex_driver/OperatingModeNotification

uint32 operating_mode
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
DeviceHandle device_handle
================================================================================
MSG: kortex_driver/Timestamp

uint32 sec
uint32 usec
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/Connection

UserProfileHandle user_handle
string connection_information
uint32 connection_identifier
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
uint32 order
")



(provide :kortex_driver/OperatingModeNotificationList "52f234be703b9c3a9743364c4bec1c58")


