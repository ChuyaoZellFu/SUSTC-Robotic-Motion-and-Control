;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MappingInfoNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MappingInfoNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPPINGINFONOTIFICATION")
  (make-package "KORTEX_DRIVER::MAPPINGINFONOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude MappingInfoNotification.msg.html


(defclass kortex_driver::MappingInfoNotification
  :super ros::object
  :slots (_controller_identifier _active_map_handle _timestamp _user_handle _connection _mapping_handle ))

(defmethod kortex_driver::MappingInfoNotification
  (:init
   (&key
    ((:controller_identifier __controller_identifier) 0)
    ((:active_map_handle __active_map_handle) (instance kortex_driver::MapHandle :init))
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    ((:mapping_handle __mapping_handle) (instance kortex_driver::MappingHandle :init))
    )
   (send-super :init)
   (setq _controller_identifier (round __controller_identifier))
   (setq _active_map_handle __active_map_handle)
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _connection __connection)
   (setq _mapping_handle __mapping_handle)
   self)
  (:controller_identifier
   (&optional __controller_identifier)
   (if __controller_identifier (setq _controller_identifier __controller_identifier)) _controller_identifier)
  (:active_map_handle
   (&rest __active_map_handle)
   (if (keywordp (car __active_map_handle))
       (send* _active_map_handle __active_map_handle)
     (progn
       (if __active_map_handle (setq _active_map_handle (car __active_map_handle)))
       _active_map_handle)))
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
  (:mapping_handle
   (&rest __mapping_handle)
   (if (keywordp (car __mapping_handle))
       (send* _mapping_handle __mapping_handle)
     (progn
       (if __mapping_handle (setq _mapping_handle (car __mapping_handle)))
       _mapping_handle)))
  (:serialization-length
   ()
   (+
    ;; uint32 _controller_identifier
    4
    ;; kortex_driver/MapHandle _active_map_handle
    (send _active_map_handle :serialization-length)
    ;; kortex_driver/Timestamp _timestamp
    (send _timestamp :serialization-length)
    ;; kortex_driver/UserProfileHandle _user_handle
    (send _user_handle :serialization-length)
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ;; kortex_driver/MappingHandle _mapping_handle
    (send _mapping_handle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _controller_identifier
       (write-long _controller_identifier s)
     ;; kortex_driver/MapHandle _active_map_handle
       (send _active_map_handle :serialize s)
     ;; kortex_driver/Timestamp _timestamp
       (send _timestamp :serialize s)
     ;; kortex_driver/UserProfileHandle _user_handle
       (send _user_handle :serialize s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;; kortex_driver/MappingHandle _mapping_handle
       (send _mapping_handle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _controller_identifier
     (setq _controller_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/MapHandle _active_map_handle
     (send _active_map_handle :deserialize buf ptr-) (incf ptr- (send _active_map_handle :serialization-length))
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;; kortex_driver/MappingHandle _mapping_handle
     (send _mapping_handle :deserialize buf ptr-) (incf ptr- (send _mapping_handle :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::MappingInfoNotification :md5sum-) "c0d2df9a9b1143d371ae7438ebf20d09")
(setf (get kortex_driver::MappingInfoNotification :datatype-) "kortex_driver/MappingInfoNotification")
(setf (get kortex_driver::MappingInfoNotification :definition-)
      "
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



(provide :kortex_driver/MappingInfoNotification "c0d2df9a9b1143d371ae7438ebf20d09")


