;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ArmStateNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ArmStateNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ARMSTATENOTIFICATION")
  (make-package "KORTEX_DRIVER::ARMSTATENOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude ArmStateNotification.msg.html


(defclass kortex_driver::ArmStateNotification
  :super ros::object
  :slots (_active_state _timestamp _connection ))

(defmethod kortex_driver::ArmStateNotification
  (:init
   (&key
    ((:active_state __active_state) 0)
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    )
   (send-super :init)
   (setq _active_state (round __active_state))
   (setq _timestamp __timestamp)
   (setq _connection __connection)
   self)
  (:active_state
   (&optional __active_state)
   (if __active_state (setq _active_state __active_state)) _active_state)
  (:timestamp
   (&rest __timestamp)
   (if (keywordp (car __timestamp))
       (send* _timestamp __timestamp)
     (progn
       (if __timestamp (setq _timestamp (car __timestamp)))
       _timestamp)))
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
    ;; uint32 _active_state
    4
    ;; kortex_driver/Timestamp _timestamp
    (send _timestamp :serialization-length)
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _active_state
       (write-long _active_state s)
     ;; kortex_driver/Timestamp _timestamp
       (send _timestamp :serialize s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _active_state
     (setq _active_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ArmStateNotification :md5sum-) "73ee66f2cc8dfa1e59b9413c6d13ee3a")
(setf (get kortex_driver::ArmStateNotification :datatype-) "kortex_driver/ArmStateNotification")
(setf (get kortex_driver::ArmStateNotification :definition-)
      "
uint32 active_state
Timestamp timestamp
Connection connection
================================================================================
MSG: kortex_driver/Timestamp

uint32 sec
uint32 usec
================================================================================
MSG: kortex_driver/Connection

UserProfileHandle user_handle
string connection_information
uint32 connection_identifier
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/ArmStateNotification "73ee66f2cc8dfa1e59b9413c6d13ee3a")


