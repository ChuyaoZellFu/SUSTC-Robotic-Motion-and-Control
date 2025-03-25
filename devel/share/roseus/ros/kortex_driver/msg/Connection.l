;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Connection)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Connection (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONNECTION")
  (make-package "KORTEX_DRIVER::CONNECTION"))

(in-package "ROS")
;;//! \htmlinclude Connection.msg.html


(defclass kortex_driver::Connection
  :super ros::object
  :slots (_user_handle _connection_information _connection_identifier ))

(defmethod kortex_driver::Connection
  (:init
   (&key
    ((:user_handle __user_handle) (instance kortex_driver::UserProfileHandle :init))
    ((:connection_information __connection_information) "")
    ((:connection_identifier __connection_identifier) 0)
    )
   (send-super :init)
   (setq _user_handle __user_handle)
   (setq _connection_information (string __connection_information))
   (setq _connection_identifier (round __connection_identifier))
   self)
  (:user_handle
   (&rest __user_handle)
   (if (keywordp (car __user_handle))
       (send* _user_handle __user_handle)
     (progn
       (if __user_handle (setq _user_handle (car __user_handle)))
       _user_handle)))
  (:connection_information
   (&optional __connection_information)
   (if __connection_information (setq _connection_information __connection_information)) _connection_information)
  (:connection_identifier
   (&optional __connection_identifier)
   (if __connection_identifier (setq _connection_identifier __connection_identifier)) _connection_identifier)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/UserProfileHandle _user_handle
    (send _user_handle :serialization-length)
    ;; string _connection_information
    4 (length _connection_information)
    ;; uint32 _connection_identifier
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfileHandle _user_handle
       (send _user_handle :serialize s)
     ;; string _connection_information
       (write-long (length _connection_information) s) (princ _connection_information s)
     ;; uint32 _connection_identifier
       (write-long _connection_identifier s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfileHandle _user_handle
     (send _user_handle :deserialize buf ptr-) (incf ptr- (send _user_handle :serialization-length))
   ;; string _connection_information
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _connection_information (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _connection_identifier
     (setq _connection_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Connection :md5sum-) "149818e986cd66f3e32843abf7b3bfe5")
(setf (get kortex_driver::Connection :datatype-) "kortex_driver/Connection")
(setf (get kortex_driver::Connection :definition-)
      "
UserProfileHandle user_handle
string connection_information
uint32 connection_identifier
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/Connection "149818e986cd66f3e32843abf7b3bfe5")


