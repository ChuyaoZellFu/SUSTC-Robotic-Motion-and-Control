;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ArmStateInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ArmStateInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ARMSTATEINFORMATION")
  (make-package "KORTEX_DRIVER::ARMSTATEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude ArmStateInformation.msg.html


(defclass kortex_driver::ArmStateInformation
  :super ros::object
  :slots (_active_state _connection ))

(defmethod kortex_driver::ArmStateInformation
  (:init
   (&key
    ((:active_state __active_state) 0)
    ((:connection __connection) (instance kortex_driver::Connection :init))
    )
   (send-super :init)
   (setq _active_state (round __active_state))
   (setq _connection __connection)
   self)
  (:active_state
   (&optional __active_state)
   (if __active_state (setq _active_state __active_state)) _active_state)
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
    ;; kortex_driver/Connection _connection
    (send _connection :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _active_state
       (write-long _active_state s)
     ;; kortex_driver/Connection _connection
       (send _connection :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _active_state
     (setq _active_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/Connection _connection
     (send _connection :deserialize buf ptr-) (incf ptr- (send _connection :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ArmStateInformation :md5sum-) "dae937682c51c5ef4ccc506f80df3c9f")
(setf (get kortex_driver::ArmStateInformation :datatype-) "kortex_driver/ArmStateInformation")
(setf (get kortex_driver::ArmStateInformation :definition-)
      "
uint32 active_state
Connection connection
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



(provide :kortex_driver/ArmStateInformation "dae937682c51c5ef4ccc506f80df3c9f")


