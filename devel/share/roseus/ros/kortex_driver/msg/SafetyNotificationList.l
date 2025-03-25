;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYNOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::SAFETYNOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude SafetyNotificationList.msg.html


(defclass kortex_driver::SafetyNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::SafetyNotificationList
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
    ;; kortex_driver/SafetyNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::SafetyNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::SafetyNotificationList :md5sum-) "c064ceaeac74b1cd3d7784c6defce6b7")
(setf (get kortex_driver::SafetyNotificationList :datatype-) "kortex_driver/SafetyNotificationList")
(setf (get kortex_driver::SafetyNotificationList :definition-)
      "
SafetyNotification[] notifications
================================================================================
MSG: kortex_driver/SafetyNotification

SafetyHandle safety_handle
uint32 value
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
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



(provide :kortex_driver/SafetyNotificationList "c064ceaeac74b1cd3d7784c6defce6b7")


