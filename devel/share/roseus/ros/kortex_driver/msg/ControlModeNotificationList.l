;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlModeNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlModeNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLMODENOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::CONTROLMODENOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude ControlModeNotificationList.msg.html


(defclass kortex_driver::ControlModeNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::ControlModeNotificationList
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
    ;; kortex_driver/Base_ControlModeNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_ControlModeNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_ControlModeNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::Base_ControlModeNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ControlModeNotificationList :md5sum-) "15ea06dcb7be562d13fce4a4e5f177b9")
(setf (get kortex_driver::ControlModeNotificationList :datatype-) "kortex_driver/ControlModeNotificationList")
(setf (get kortex_driver::ControlModeNotificationList :definition-)
      "
Base_ControlModeNotification[] notifications
================================================================================
MSG: kortex_driver/Base_ControlModeNotification

uint32 control_mode
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
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



(provide :kortex_driver/ControlModeNotificationList "15ea06dcb7be562d13fce4a4e5f177b9")


