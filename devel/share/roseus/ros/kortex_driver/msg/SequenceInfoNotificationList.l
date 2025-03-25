;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SequenceInfoNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SequenceInfoNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SEQUENCEINFONOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::SEQUENCEINFONOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude SequenceInfoNotificationList.msg.html


(defclass kortex_driver::SequenceInfoNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::SequenceInfoNotificationList
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
    ;; kortex_driver/SequenceInfoNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceInfoNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceInfoNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::SequenceInfoNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::SequenceInfoNotificationList :md5sum-) "064d50868af0284b9e6fb269565cff60")
(setf (get kortex_driver::SequenceInfoNotificationList :datatype-) "kortex_driver/SequenceInfoNotificationList")
(setf (get kortex_driver::SequenceInfoNotificationList :definition-)
      "
SequenceInfoNotification[] notifications
================================================================================
MSG: kortex_driver/SequenceInfoNotification

uint32 event_identifier
SequenceHandle sequence_handle
uint32 task_index
uint32 group_identifier
Timestamp timestamp
UserProfileHandle user_handle
uint32 abort_details
Connection connection
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
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



(provide :kortex_driver/SequenceInfoNotificationList "064d50868af0284b9e6fb269565cff60")


