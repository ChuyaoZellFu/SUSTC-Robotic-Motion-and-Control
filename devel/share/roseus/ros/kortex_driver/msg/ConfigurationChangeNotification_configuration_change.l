;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConfigurationChangeNotification_configuration_change)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConfigurationChangeNotification_configuration_change (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONFIGURATIONCHANGENOTIFICATION_CONFIGURATION_CHANGE")
  (make-package "KORTEX_DRIVER::CONFIGURATIONCHANGENOTIFICATION_CONFIGURATION_CHANGE"))

(in-package "ROS")
;;//! \htmlinclude ConfigurationChangeNotification_configuration_change.msg.html


(defclass kortex_driver::ConfigurationChangeNotification_configuration_change
  :super ros::object
  :slots (_sequence_handle _action_handle _mapping_handle _map_group_handle _map_handle _user_profile_handle _protection_zone_handle _safety_handle _network_handle _ssid _controller_handle ))

(defmethod kortex_driver::ConfigurationChangeNotification_configuration_change
  (:init
   (&key
    ((:sequence_handle __sequence_handle) ())
    ((:action_handle __action_handle) ())
    ((:mapping_handle __mapping_handle) ())
    ((:map_group_handle __map_group_handle) ())
    ((:map_handle __map_handle) ())
    ((:user_profile_handle __user_profile_handle) ())
    ((:protection_zone_handle __protection_zone_handle) ())
    ((:safety_handle __safety_handle) ())
    ((:network_handle __network_handle) ())
    ((:ssid __ssid) ())
    ((:controller_handle __controller_handle) ())
    )
   (send-super :init)
   (setq _sequence_handle __sequence_handle)
   (setq _action_handle __action_handle)
   (setq _mapping_handle __mapping_handle)
   (setq _map_group_handle __map_group_handle)
   (setq _map_handle __map_handle)
   (setq _user_profile_handle __user_profile_handle)
   (setq _protection_zone_handle __protection_zone_handle)
   (setq _safety_handle __safety_handle)
   (setq _network_handle __network_handle)
   (setq _ssid __ssid)
   (setq _controller_handle __controller_handle)
   self)
  (:sequence_handle
   (&rest __sequence_handle)
   (if (keywordp (car __sequence_handle))
       (send* _sequence_handle __sequence_handle)
     (progn
       (if __sequence_handle (setq _sequence_handle (car __sequence_handle)))
       _sequence_handle)))
  (:action_handle
   (&rest __action_handle)
   (if (keywordp (car __action_handle))
       (send* _action_handle __action_handle)
     (progn
       (if __action_handle (setq _action_handle (car __action_handle)))
       _action_handle)))
  (:mapping_handle
   (&rest __mapping_handle)
   (if (keywordp (car __mapping_handle))
       (send* _mapping_handle __mapping_handle)
     (progn
       (if __mapping_handle (setq _mapping_handle (car __mapping_handle)))
       _mapping_handle)))
  (:map_group_handle
   (&rest __map_group_handle)
   (if (keywordp (car __map_group_handle))
       (send* _map_group_handle __map_group_handle)
     (progn
       (if __map_group_handle (setq _map_group_handle (car __map_group_handle)))
       _map_group_handle)))
  (:map_handle
   (&rest __map_handle)
   (if (keywordp (car __map_handle))
       (send* _map_handle __map_handle)
     (progn
       (if __map_handle (setq _map_handle (car __map_handle)))
       _map_handle)))
  (:user_profile_handle
   (&rest __user_profile_handle)
   (if (keywordp (car __user_profile_handle))
       (send* _user_profile_handle __user_profile_handle)
     (progn
       (if __user_profile_handle (setq _user_profile_handle (car __user_profile_handle)))
       _user_profile_handle)))
  (:protection_zone_handle
   (&rest __protection_zone_handle)
   (if (keywordp (car __protection_zone_handle))
       (send* _protection_zone_handle __protection_zone_handle)
     (progn
       (if __protection_zone_handle (setq _protection_zone_handle (car __protection_zone_handle)))
       _protection_zone_handle)))
  (:safety_handle
   (&rest __safety_handle)
   (if (keywordp (car __safety_handle))
       (send* _safety_handle __safety_handle)
     (progn
       (if __safety_handle (setq _safety_handle (car __safety_handle)))
       _safety_handle)))
  (:network_handle
   (&rest __network_handle)
   (if (keywordp (car __network_handle))
       (send* _network_handle __network_handle)
     (progn
       (if __network_handle (setq _network_handle (car __network_handle)))
       _network_handle)))
  (:ssid
   (&rest __ssid)
   (if (keywordp (car __ssid))
       (send* _ssid __ssid)
     (progn
       (if __ssid (setq _ssid (car __ssid)))
       _ssid)))
  (:controller_handle
   (&rest __controller_handle)
   (if (keywordp (car __controller_handle))
       (send* _controller_handle __controller_handle)
     (progn
       (if __controller_handle (setq _controller_handle (car __controller_handle)))
       _controller_handle)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SequenceHandle[] _sequence_handle
    (apply #'+ (send-all _sequence_handle :serialization-length)) 4
    ;; kortex_driver/ActionHandle[] _action_handle
    (apply #'+ (send-all _action_handle :serialization-length)) 4
    ;; kortex_driver/MappingHandle[] _mapping_handle
    (apply #'+ (send-all _mapping_handle :serialization-length)) 4
    ;; kortex_driver/MapGroupHandle[] _map_group_handle
    (apply #'+ (send-all _map_group_handle :serialization-length)) 4
    ;; kortex_driver/MapHandle[] _map_handle
    (apply #'+ (send-all _map_handle :serialization-length)) 4
    ;; kortex_driver/UserProfileHandle[] _user_profile_handle
    (apply #'+ (send-all _user_profile_handle :serialization-length)) 4
    ;; kortex_driver/ProtectionZoneHandle[] _protection_zone_handle
    (apply #'+ (send-all _protection_zone_handle :serialization-length)) 4
    ;; kortex_driver/SafetyHandle[] _safety_handle
    (apply #'+ (send-all _safety_handle :serialization-length)) 4
    ;; kortex_driver/NetworkHandle[] _network_handle
    (apply #'+ (send-all _network_handle :serialization-length)) 4
    ;; kortex_driver/Ssid[] _ssid
    (apply #'+ (send-all _ssid :serialization-length)) 4
    ;; kortex_driver/ControllerHandle[] _controller_handle
    (apply #'+ (send-all _controller_handle :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceHandle[] _sequence_handle
     (write-long (length _sequence_handle) s)
     (dolist (elem _sequence_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/ActionHandle[] _action_handle
     (write-long (length _action_handle) s)
     (dolist (elem _action_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/MappingHandle[] _mapping_handle
     (write-long (length _mapping_handle) s)
     (dolist (elem _mapping_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/MapGroupHandle[] _map_group_handle
     (write-long (length _map_group_handle) s)
     (dolist (elem _map_group_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/MapHandle[] _map_handle
     (write-long (length _map_handle) s)
     (dolist (elem _map_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/UserProfileHandle[] _user_profile_handle
     (write-long (length _user_profile_handle) s)
     (dolist (elem _user_profile_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/ProtectionZoneHandle[] _protection_zone_handle
     (write-long (length _protection_zone_handle) s)
     (dolist (elem _protection_zone_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/SafetyHandle[] _safety_handle
     (write-long (length _safety_handle) s)
     (dolist (elem _safety_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/NetworkHandle[] _network_handle
     (write-long (length _network_handle) s)
     (dolist (elem _network_handle)
       (send elem :serialize s)
       )
     ;; kortex_driver/Ssid[] _ssid
     (write-long (length _ssid) s)
     (dolist (elem _ssid)
       (send elem :serialize s)
       )
     ;; kortex_driver/ControllerHandle[] _controller_handle
     (write-long (length _controller_handle) s)
     (dolist (elem _controller_handle)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceHandle[] _sequence_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sequence_handle (let (r) (dotimes (i n) (push (instance kortex_driver::SequenceHandle :init) r)) r))
     (dolist (elem- _sequence_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ActionHandle[] _action_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _action_handle (let (r) (dotimes (i n) (push (instance kortex_driver::ActionHandle :init) r)) r))
     (dolist (elem- _action_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/MappingHandle[] _mapping_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mapping_handle (let (r) (dotimes (i n) (push (instance kortex_driver::MappingHandle :init) r)) r))
     (dolist (elem- _mapping_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/MapGroupHandle[] _map_group_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_group_handle (let (r) (dotimes (i n) (push (instance kortex_driver::MapGroupHandle :init) r)) r))
     (dolist (elem- _map_group_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/MapHandle[] _map_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_handle (let (r) (dotimes (i n) (push (instance kortex_driver::MapHandle :init) r)) r))
     (dolist (elem- _map_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/UserProfileHandle[] _user_profile_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _user_profile_handle (let (r) (dotimes (i n) (push (instance kortex_driver::UserProfileHandle :init) r)) r))
     (dolist (elem- _user_profile_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ProtectionZoneHandle[] _protection_zone_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _protection_zone_handle (let (r) (dotimes (i n) (push (instance kortex_driver::ProtectionZoneHandle :init) r)) r))
     (dolist (elem- _protection_zone_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/SafetyHandle[] _safety_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _safety_handle (let (r) (dotimes (i n) (push (instance kortex_driver::SafetyHandle :init) r)) r))
     (dolist (elem- _safety_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/NetworkHandle[] _network_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _network_handle (let (r) (dotimes (i n) (push (instance kortex_driver::NetworkHandle :init) r)) r))
     (dolist (elem- _network_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/Ssid[] _ssid
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ssid (let (r) (dotimes (i n) (push (instance kortex_driver::Ssid :init) r)) r))
     (dolist (elem- _ssid)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ControllerHandle[] _controller_handle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controller_handle (let (r) (dotimes (i n) (push (instance kortex_driver::ControllerHandle :init) r)) r))
     (dolist (elem- _controller_handle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ConfigurationChangeNotification_configuration_change :md5sum-) "da6a00bf2364ba1e44afbbbd7c77db5b")
(setf (get kortex_driver::ConfigurationChangeNotification_configuration_change :datatype-) "kortex_driver/ConfigurationChangeNotification_configuration_change")
(setf (get kortex_driver::ConfigurationChangeNotification_configuration_change :definition-)
      "
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
MSG: kortex_driver/UserProfileHandle

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



(provide :kortex_driver/ConfigurationChangeNotification_configuration_change "da6a00bf2364ba1e44afbbbd7c77db5b")


