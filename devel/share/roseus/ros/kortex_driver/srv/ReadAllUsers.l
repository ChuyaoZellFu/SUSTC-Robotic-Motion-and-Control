;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadAllUsers)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadAllUsers (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READALLUSERS")
  (make-package "KORTEX_DRIVER::READALLUSERS"))
(unless (find-package "KORTEX_DRIVER::READALLUSERSREQUEST")
  (make-package "KORTEX_DRIVER::READALLUSERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::READALLUSERSRESPONSE")
  (make-package "KORTEX_DRIVER::READALLUSERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadAllUsersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadAllUsersRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadAllUsersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadAllUsersResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::UserList :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/UserList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadAllUsers
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadAllUsers :md5sum-) "f4e7c3e24386fc73fa17c3ea4d2a06c5")
(setf (get kortex_driver::ReadAllUsers :datatype-) "kortex_driver/ReadAllUsers")
(setf (get kortex_driver::ReadAllUsers :request) kortex_driver::ReadAllUsersRequest)
(setf (get kortex_driver::ReadAllUsers :response) kortex_driver::ReadAllUsersResponse)

(defmethod kortex_driver::ReadAllUsersRequest
  (:response () (instance kortex_driver::ReadAllUsersResponse :init)))

(setf (get kortex_driver::ReadAllUsersRequest :md5sum-) "f4e7c3e24386fc73fa17c3ea4d2a06c5")
(setf (get kortex_driver::ReadAllUsersRequest :datatype-) "kortex_driver/ReadAllUsersRequest")
(setf (get kortex_driver::ReadAllUsersRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
UserList output

================================================================================
MSG: kortex_driver/UserList

UserProfileHandle[] user_handles
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")

(setf (get kortex_driver::ReadAllUsersResponse :md5sum-) "f4e7c3e24386fc73fa17c3ea4d2a06c5")
(setf (get kortex_driver::ReadAllUsersResponse :datatype-) "kortex_driver/ReadAllUsersResponse")
(setf (get kortex_driver::ReadAllUsersResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
UserList output

================================================================================
MSG: kortex_driver/UserList

UserProfileHandle[] user_handles
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")



(provide :kortex_driver/ReadAllUsers "f4e7c3e24386fc73fa17c3ea4d2a06c5")


