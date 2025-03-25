;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConfigurationChangeNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConfigurationChangeNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONFIGURATIONCHANGENOTIFICATION")
  (make-package "KORTEX_DRIVER::CONFIGURATIONCHANGENOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude ConfigurationChangeNotification.msg.html


(defclass kortex_driver::ConfigurationChangeNotification
  :super ros::object
  :slots (_event _timestamp _user_handle _connection _oneof_configuration_change ))

(defmethod kortex_driver::ConfigurationChangeNotification
  (:init
   (&key
    ((:event __event) 0)
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    ((:oneof_configuration_change __oneof_configuration_change) (instance kortex_driver::ConfigurationChangeNotification_configuration_change :init))
    )
   (send-super :init)
   (setq _event (round __event))
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _connection __connection)
   (setq _oneof_configuration_change __oneof_configuration_change)
   self)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:timestamp
   (&rest __timestamp)
   (if (keywordp (car __timestamp))
       (send* _timestamp __timestamp)
     (progn
       (if __timestamp (setq _timestamp (car __timestamp)))
       _timestamp)))
  (:user_handle
   (&rest __user_handle)
   (if (keywordp (car __user_handle))
       (send* _user_handle __user_handle)
     (progn
       (if __user_handle (setq _user_handle (car __user_handle)))
       _user_handle)))
  (:connection
   (&rest __connection)
   (if (keywordp (car __connection))
       (send* _connection __connection)
     (progn
       (if __connection (setq _connection (car __connection)))
       _connection)))
  (:oneof_configuration_change
   (&rest __oneof_configuration_change)
   (if (keywordp (car __oneof_configuration_change))
       (send* _oneof_configuration_change __oneof_configuration_change)
     (progn
       (if __oneof_configuration_change (setq _oneof_configuration_change (car __oneof_configuration_change)))
       _oneof_configuration_change)))
  (:serialization-length
   ()
   (+
    ;; uint32 _event
    4
    ;; kortex_driver/Timestamp _timestamp
    (send _timestamp :serialization-length)
    ;; kortex_driver/UserProfileHandle _user_handle
    (send _user_handle :serialization-length)
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ;; kortex_driver/ConfigurationChangeNotification_configuration_change _oneof_configuration_change
    (send _oneof_configuration_change :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _event
       (write-long _event s)
     ;; kortex_driver/Timestamp _timestamp
       (send _timestamp :serialize s)
     ;; kortex_driver/UserProfileHandle _user_handle
       (send _user_handle :serialize s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;; kortex_driver/ConfigurationChangeNotification_configuration_change _oneof_configuration_change
       (send _oneof_configuration_change :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _event
     (setq _event (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;; kortex_driver/ConfigurationChangeNotification_configuration_change _oneof_configuration_change
     (send _oneof_configuration_change :deserialize buf ptr-) (incf ptr- (send _oneof_configuration_change :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ConfigurationChangeNotification :md5sum-) "3adec35b48ab707b96b990bb86718c1e")
(setf (get kortex_driver::ConfigurationChangeNotification :datatype-) "kortex_driver/ConfigurationChangeNotification")
(setf (get kortex_driver::ConfigurationChangeNotification :definition-)
      "
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



(provide :kortex_driver/ConfigurationChangeNotification "3adec35b48ab707b96b990bb86718c1e")


