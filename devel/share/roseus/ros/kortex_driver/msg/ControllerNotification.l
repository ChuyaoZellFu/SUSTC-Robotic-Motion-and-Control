;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERNOTIFICATION")
  (make-package "KORTEX_DRIVER::CONTROLLERNOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude ControllerNotification.msg.html


(defclass kortex_driver::ControllerNotification
  :super ros::object
  :slots (_timestamp _user_handle _connection _oneof_state ))

(defmethod kortex_driver::ControllerNotification
  (:init
   (&key
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    ((:oneof_state __oneof_state) (instance kortex_driver::ControllerNotification_state :init))
    )
   (send-super :init)
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _connection __connection)
   (setq _oneof_state __oneof_state)
   self)
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
  (:oneof_state
   (&rest __oneof_state)
   (if (keywordp (car __oneof_state))
       (send* _oneof_state __oneof_state)
     (progn
       (if __oneof_state (setq _oneof_state (car __oneof_state)))
       _oneof_state)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Timestamp _timestamp
    (send _timestamp :serialization-length)
    ;; kortex_driver/UserProfileHandle _user_handle
    (send _user_handle :serialization-length)
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ;; kortex_driver/ControllerNotification_state _oneof_state
    (send _oneof_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Timestamp _timestamp
       (send _timestamp :serialize s)
     ;; kortex_driver/UserProfileHandle _user_handle
       (send _user_handle :serialize s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;; kortex_driver/ControllerNotification_state _oneof_state
       (send _oneof_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;; kortex_driver/ControllerNotification_state _oneof_state
     (send _oneof_state :deserialize buf ptr-) (incf ptr- (send _oneof_state :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerNotification :md5sum-) "4a8a872ea40ec244a141a390ef3880ea")
(setf (get kortex_driver::ControllerNotification :datatype-) "kortex_driver/ControllerNotification")
(setf (get kortex_driver::ControllerNotification :definition-)
      "
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



(provide :kortex_driver/ControllerNotification "4a8a872ea40ec244a141a390ef3880ea")


