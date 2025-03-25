;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ServoingModeNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ServoingModeNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SERVOINGMODENOTIFICATION")
  (make-package "KORTEX_DRIVER::SERVOINGMODENOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude ServoingModeNotification.msg.html


(defclass kortex_driver::ServoingModeNotification
  :super ros::object
  :slots (_servoing_mode _timestamp _user_handle _connection ))

(defmethod kortex_driver::ServoingModeNotification
  (:init
   (&key
    ((:servoing_mode __servoing_mode) 0)
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    )
   (send-super :init)
   (setq _servoing_mode (round __servoing_mode))
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _connection __connection)
   self)
  (:servoing_mode
   (&optional __servoing_mode)
   (if __servoing_mode (setq _servoing_mode __servoing_mode)) _servoing_mode)
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
    ;; uint32 _servoing_mode
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
     ;; uint32 _servoing_mode
       (write-long _servoing_mode s)
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
   ;; uint32 _servoing_mode
     (setq _servoing_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ServoingModeNotification :md5sum-) "cbaaac12490fcae846b29cc0756b4494")
(setf (get kortex_driver::ServoingModeNotification :datatype-) "kortex_driver/ServoingModeNotification")
(setf (get kortex_driver::ServoingModeNotification :definition-)
      "
uint32 servoing_mode
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
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



(provide :kortex_driver/ServoingModeNotification "cbaaac12490fcae846b29cc0756b4494")


