;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYNOTIFICATION")
  (make-package "KORTEX_DRIVER::SAFETYNOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude SafetyNotification.msg.html


(defclass kortex_driver::SafetyNotification
  :super ros::object
  :slots (_safety_handle _value _timestamp _user_handle _connection ))

(defmethod kortex_driver::SafetyNotification
  (:init
   (&key
    ((:safety_handle __safety_handle) (instance kortex_driver::SafetyHandle :init))
    ((:value __value) 0)
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    )
   (send-super :init)
   (setq _safety_handle __safety_handle)
   (setq _value (round __value))
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _connection __connection)
   self)
  (:safety_handle
   (&rest __safety_handle)
   (if (keywordp (car __safety_handle))
       (send* _safety_handle __safety_handle)
     (progn
       (if __safety_handle (setq _safety_handle (car __safety_handle)))
       _safety_handle)))
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
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
    ;; kortex_driver/SafetyHandle _safety_handle
    (send _safety_handle :serialization-length)
    ;; uint32 _value
    4
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
     ;; kortex_driver/SafetyHandle _safety_handle
       (send _safety_handle :serialize s)
     ;; uint32 _value
       (write-long _value s)
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
   ;; kortex_driver/SafetyHandle _safety_handle
     (send _safety_handle :deserialize buf ptr-) (incf ptr- (send _safety_handle :serialization-length))
   ;; uint32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::SafetyNotification :md5sum-) "dffadcb533e64106d087a7e7e2548f72")
(setf (get kortex_driver::SafetyNotification :datatype-) "kortex_driver/SafetyNotification")
(setf (get kortex_driver::SafetyNotification :definition-)
      "
SafetyHandle safety_handle
uint32 value
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
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



(provide :kortex_driver/SafetyNotification "dffadcb533e64106d087a7e7e2548f72")


