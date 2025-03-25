;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RobotEventNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RobotEventNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ROBOTEVENTNOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::ROBOTEVENTNOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude RobotEventNotificationList.msg.html


(defclass kortex_driver::RobotEventNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::RobotEventNotificationList
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
    ;; kortex_driver/RobotEventNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/RobotEventNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/RobotEventNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::RobotEventNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::RobotEventNotificationList :md5sum-) "31f7be45a79909d8e884dbb37990a7f2")
(setf (get kortex_driver::RobotEventNotificationList :datatype-) "kortex_driver/RobotEventNotificationList")
(setf (get kortex_driver::RobotEventNotificationList :definition-)
      "
RobotEventNotification[] notifications
================================================================================
MSG: kortex_driver/RobotEventNotification

uint32 event
DeviceHandle handle
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
uint32 order
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
")



(provide :kortex_driver/RobotEventNotificationList "31f7be45a79909d8e884dbb37990a7f2")


