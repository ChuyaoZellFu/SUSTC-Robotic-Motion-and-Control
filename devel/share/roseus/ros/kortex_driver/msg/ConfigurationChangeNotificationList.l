;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConfigurationChangeNotificationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConfigurationChangeNotificationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONFIGURATIONCHANGENOTIFICATIONLIST")
  (make-package "KORTEX_DRIVER::CONFIGURATIONCHANGENOTIFICATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude ConfigurationChangeNotificationList.msg.html


(defclass kortex_driver::ConfigurationChangeNotificationList
  :super ros::object
  :slots (_notifications ))

(defmethod kortex_driver::ConfigurationChangeNotificationList
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
    ;; kortex_driver/ConfigurationChangeNotification[] _notifications
    (apply #'+ (send-all _notifications :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ConfigurationChangeNotification[] _notifications
     (write-long (length _notifications) s)
     (dolist (elem _notifications)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ConfigurationChangeNotification[] _notifications
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _notifications (let (r) (dotimes (i n) (push (instance kortex_driver::ConfigurationChangeNotification :init) r)) r))
     (dolist (elem- _notifications)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ConfigurationChangeNotificationList :md5sum-) "bf8ec7af96a2de99f1b33e2bf4146af1")
(setf (get kortex_driver::ConfigurationChangeNotificationList :datatype-) "kortex_driver/ConfigurationChangeNotificationList")
(setf (get kortex_driver::ConfigurationChangeNotificationList :definition-)
      "
ConfigurationChangeNotification[] notifications
================================================================================
MSG: kortex_driver/ConfigurationChangeNotification

uint32 event
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
ConfigurationChangeNotification_configuration_change oneof_configuration_change
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
MSG: kortex_driver/ConfigurationChangeNotification_configuration_change

SequenceHandle[] sequence_handle
ActionHandle[] action_handle
MappingHandle[] mapping_handle
MapGroupHandle[] map_group_handle
MapHandle[] map_handle
UserProfileHandle[] user_profile_handle
ProtectionZoneHandle[] protection_zone_handle
SafetyHandle[] safety_handle
NetworkHandle[] network_handle
Ssid[] ssid
ControllerHandle[] controller_handle
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/ActionHandle

uint32 identifier
uint32 action_type
uint32 permission
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapGroupHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
================================================================================
MSG: kortex_driver/NetworkHandle

uint32 type
================================================================================
MSG: kortex_driver/Ssid

string identifier
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
")



(provide :kortex_driver/ConfigurationChangeNotificationList "bf8ec7af96a2de99f1b33e2bf4146af1")


