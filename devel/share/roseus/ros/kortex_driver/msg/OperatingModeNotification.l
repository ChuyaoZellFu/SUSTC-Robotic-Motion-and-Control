;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OperatingModeNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OperatingModeNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPERATINGMODENOTIFICATION")
  (make-package "KORTEX_DRIVER::OPERATINGMODENOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude OperatingModeNotification.msg.html


(defclass kortex_driver::OperatingModeNotification
  :super ros::object
  :slots (_operating_mode _timestamp _user_handle _connection _device_handle ))

(defmethod kortex_driver::OperatingModeNotification
  (:init
   (&key
    ((:operating_mode __operating_mode) 0)
    ((:timestamp __timestamp) (instance kortex_driver::Timestamp :init))
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection __connection) (instance kortex_driver::Connection :init))
    ((:device_handle __device_handle) (instance kortex_driver::DeviceHandle :init))
    )
   (send-super :init)
   (setq _operating_mode (round __operating_mode))
   (setq _timestamp __timestamp)
   (setq _user_handle __user_handle)
   (setq _connection __connection)
   (setq _device_handle __device_handle)
   self)
  (:operating_mode
   (&optional __operating_mode)
   (if __operating_mode (setq _operating_mode __operating_mode)) _operating_mode)
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
  (:device_handle
   (&rest __device_handle)
   (if (keywordp (car __device_handle))
       (send* _device_handle __device_handle)
     (progn
       (if __device_handle (setq _device_handle (car __device_handle)))
       _device_handle)))
  (:serialization-length
   ()
   (+
    ;; uint32 _operating_mode
    4
    ;; kortex_driver/Timestamp _timestamp
    (send _timestamp :serialization-length)
    ;; kortex_driver/UserProfileHandle _user_handle
    (send _user_handle :serialization-length)
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ;; kortex_driver/DeviceHandle _device_handle
    (send _device_handle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _operating_mode
       (write-long _operating_mode s)
     ;; kortex_driver/Timestamp _timestamp
       (send _timestamp :serialize s)
     ;; kortex_driver/UserProfileHandle _user_handle
       (send _user_handle :serialize s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;; kortex_driver/DeviceHandle _device_handle
       (send _device_handle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _operating_mode
     (setq _operating_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Timestamp _timestamp
     (send _timestamp :deserialize buf ptr-) (incf ptr- (send _timestamp :serialization-length))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;; kortex_driver/DeviceHandle _device_handle
     (send _device_handle :deserialize buf ptr-) (incf ptr- (send _device_handle :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::OperatingModeNotification :md5sum-) "3307c63e67ebe8d3a49769532d9dee31")
(setf (get kortex_driver::OperatingModeNotification :datatype-) "kortex_driver/OperatingModeNotification")
(setf (get kortex_driver::OperatingModeNotification :definition-)
      "
uint32 operating_mode
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
DeviceHandle device_handle
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
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
uint32 order
")



(provide :kortex_driver/OperatingModeNotification "3307c63e67ebe8d3a49769532d9dee31")


