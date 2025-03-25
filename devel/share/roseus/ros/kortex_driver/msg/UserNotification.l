;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UserNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UserNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::USERNOTIFICATION")
  (make-package "KORTEX_DRIVER::USERNOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude UserNotification.msg.html


(defclass kortex_driver::UserNotification
  :super ros::object
  :slots (_user_event _modified_user _timestamp _user_handle _connection ))

(defmethod kortex_driver::UserNotification
  (:init
   (&key
    ((:user_event __user_event) 0)
    ((:modified_user __modified_user) (instance kortex_driver::UserProfileHandle :init))
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    )
   (send-super :init)
   (setq _user_event (round __user_event))
   (setq _modified_user __modified_user)
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _connection __connection)
   self)
  (:user_event
   (&optional __user_event)
   (if __user_event (setq _user_event __user_event)) _user_event)
  (:modified_user
   (&rest __modified_user)
   (if (keywordp (car __modified_user))
       (send* _modified_user __modified_user)
     (progn
       (if __modified_user (setq _modified_user (car __modified_user)))
       _modified_user)))
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
  (:serialization-length
   ()
   (+
    ;; uint32 _user_event
    4
    ;; kortex_driver/UserProfileHandle _modified_user
    (send _modified_user :serialization-length)
    ;; kortex_driver/Timestamp _timestamp
    (send _timestamp :serialization-length)
    ;; kortex_driver/UserProfileHandle _user_handle
    (send _user_handle :serialization-length)
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _user_event
       (write-long _user_event s)
     ;; kortex_driver/UserProfileHandle _modified_user
       (send _modified_user :serialize s)
     ;; kortex_driver/Timestamp _timestamp
       (send _timestamp :serialize s)
     ;; kortex_driver/UserProfileHandle _user_handle
       (send _user_handle :serialize s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _user_event
     (setq _user_event (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/UserProfileHandle _modified_user
     (send _modified_user :deserialize buf ptr-) (incf ptr- (send _modified_user :serialization-length))
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::UserNotification :md5sum-) "deba2e21a3d1183442bf7c264989e80c")
(setf (get kortex_driver::UserNotification :datatype-) "kortex_driver/UserNotification")
(setf (get kortex_driver::UserNotification :definition-)
      "
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



(provide :kortex_driver/UserNotification "deba2e21a3d1183442bf7c264989e80c")


