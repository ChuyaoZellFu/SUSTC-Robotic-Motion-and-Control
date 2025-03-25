;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MappingInfoNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MappingInfoNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPPINGINFONOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::MAPPINGINFONOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude MappingInfoNotificationList.msg.html


(defclass kortex_driver::MappingInfoNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::MappingInfoNotificationList
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
    ;; kortex_driver/MappingInfoNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MappingInfoNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MappingInfoNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::MappingInfoNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::MappingInfoNotificationList :md5sum-) "300ba98facffe93043b4010aa2c28f88")
(setf (get kortex_driver::MappingInfoNotificationList :datatype-) "kortex_driver/MappingInfoNotificationList")
(setf (get kortex_driver::MappingInfoNotificationList :definition-)
      "
MappingInfoNotification[] notifications
================================================================================
MSG: kortex_driver/MappingInfoNotification

uint32 controller_identifier
MapHandle active_map_handle
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
MappingHandle mapping_handle
================================================================================
MSG: kortex_driver/MapHandle

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
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/MappingInfoNotificationList "300ba98facffe93043b4010aa2c28f88")


