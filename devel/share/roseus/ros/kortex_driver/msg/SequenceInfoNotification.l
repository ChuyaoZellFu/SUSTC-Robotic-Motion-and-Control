;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SequenceInfoNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SequenceInfoNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SEQUENCEINFONOTIFICATION")
  (make-package "KORTEX_DRIVER::SEQUENCEINFONOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude SequenceInfoNotification.msg.html


(defclass kortex_driver::SequenceInfoNotification
  :super ros::object
  :slots (_event_identifier _sequence_handle _task_index _group_identifier _timestamp _user_handle _abort_details _connection ))

(defmethod kortex_driver::SequenceInfoNotification
  (:init
   (&key
    ((:event_identifier __event_identifier) 0)
    ((:sequence_handle __sequence_handle) (instance kortex_driver::SequenceHandle :init))
    ((:task_index __task_index) 0)
    ((:group_identifier __group_identifier) 0)
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:abort_details __abort_details) 0)
    ((:connection __connection) (instance kortex_driver::Connection :init))
    )
   (send-super :init)
   (setq _event_identifier (round __event_identifier))
   (setq _sequence_handle __sequence_handle)
   (setq _task_index (round __task_index))
   (setq _group_identifier (round __group_identifier))
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _abort_details (round __abort_details))
   (setq _connection __connection)
   self)
  (:event_identifier
   (&optional __event_identifier)
   (if __event_identifier (setq _event_identifier __event_identifier)) _event_identifier)
  (:sequence_handle
   (&rest __sequence_handle)
   (if (keywordp (car __sequence_handle))
       (send* _sequence_handle __sequence_handle)
     (progn
       (if __sequence_handle (setq _sequence_handle (car __sequence_handle)))
       _sequence_handle)))
  (:task_index
   (&optional __task_index)
   (if __task_index (setq _task_index __task_index)) _task_index)
  (:group_identifier
   (&optional __group_identifier)
   (if __group_identifier (setq _group_identifier __group_identifier)) _group_identifier)
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
  (:abort_details
   (&optional __abort_details)
   (if __abort_details (setq _abort_details __abort_details)) _abort_details)
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
    ;; uint32 _event_identifier
    4
    ;; kortex_driver/SequenceHandle _sequence_handle
    (send _sequence_handle :serialization-length)
    ;; uint32 _task_index
    4
    ;; uint32 _group_identifier
    4
    ;; kortex_driver/Timestamp _timestamp
    (send _timestamp :serialization-length)
    ;; kortex_driver/UserProfileHandle _user_handle
    (send _user_handle :serialization-length)
    ;; uint32 _abort_details
    4
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _event_identifier
       (write-long _event_identifier s)
     ;; kortex_driver/SequenceHandle _sequence_handle
       (send _sequence_handle :serialize s)
     ;; uint32 _task_index
       (write-long _task_index s)
     ;; uint32 _group_identifier
       (write-long _group_identifier s)
     ;; kortex_driver/Timestamp _timestamp
       (send _timestamp :serialize s)
     ;; kortex_driver/UserProfileHandle _user_handle
       (send _user_handle :serialize s)
     ;; uint32 _abort_details
       (write-long _abort_details s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _event_identifier
     (setq _event_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/SequenceHandle _sequence_handle
     (send _sequence_handle :deserialize buf ptr-) (incf ptr- (send _sequence_handle :serialization-length))
   ;; uint32 _task_index
     (setq _task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _group_identifier
     (setq _group_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; uint32 _abort_details
     (setq _abort_details (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::SequenceInfoNotification :md5sum-) "cfd9f46ae5ef4976e20a716c9bac5aef")
(setf (get kortex_driver::SequenceInfoNotification :datatype-) "kortex_driver/SequenceInfoNotification")
(setf (get kortex_driver::SequenceInfoNotification :definition-)
      "
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



(provide :kortex_driver/SequenceInfoNotification "cfd9f46ae5ef4976e20a716c9bac5aef")


