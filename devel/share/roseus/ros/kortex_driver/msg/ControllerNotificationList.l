;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERNOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::CONTROLLERNOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude ControllerNotificationList.msg.html


(defclass kortex_driver::ControllerNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::ControllerNotificationList
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
    ;; kortex_driver/ControllerNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::ControllerNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerNotificationList :md5sum-) "3d9f337a9914809bd100d1e4faaea316")
(setf (get kortex_driver::ControllerNotificationList :datatype-) "kortex_driver/ControllerNotificationList")
(setf (get kortex_driver::ControllerNotificationList :definition-)
      "
ControllerNotification[] notifications
================================================================================
MSG: kortex_driver/ControllerNotification

Timestamp timestamp
UserProfileHandle user_handle
Connection connection
ControllerNotification_state oneof_state
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
MSG: kortex_driver/ControllerNotification_state

ControllerState[] controller_state
ControllerElementState[] controller_element
================================================================================
MSG: kortex_driver/ControllerState

ControllerHandle handle
uint32 event_type
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/ControllerElementState

ControllerElementHandle handle
uint32 event_type
float32 axis_value
================================================================================
MSG: kortex_driver/ControllerElementHandle

ControllerHandle controller_handle
ControllerElementHandle_identifier oneof_identifier
================================================================================
MSG: kortex_driver/ControllerElementHandle_identifier

uint32[] button
uint32[] axis
")



(provide :kortex_driver/ControllerNotificationList "3d9f337a9914809bd100d1e4faaea316")


