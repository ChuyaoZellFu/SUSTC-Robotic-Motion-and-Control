;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UserNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UserNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::USERNOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::USERNOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude UserNotificationList.msg.html


(defclass kortex_driver::UserNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::UserNotificationList
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
    ;; kortex_driver/UserNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::UserNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::UserNotificationList :md5sum-) "758e6f505a108790f7c5f9f46bc75ba3")
(setf (get kortex_driver::UserNotificationList :datatype-) "kortex_driver/UserNotificationList")
(setf (get kortex_driver::UserNotificationList :definition-)
      "
UserNotification[] notifications
================================================================================
MSG: kortex_driver/UserNotification

uint32 user_event
UserProfileHandle modified_user
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/Timestamp

uint32 sec
uint32 usec
================================================================================
MSG: kortex_driver/Connection

UserProfileHandle user_handle
string connection_information
uint32 connection_identifier
")



(provide :kortex_driver/UserNotificationList "758e6f505a108790f7c5f9f46bc75ba3")


