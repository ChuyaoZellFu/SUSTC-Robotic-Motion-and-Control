;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadUserProfile)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadUserProfile (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READUSERPROFILE")
  (make-package "KORTEX_DRIVER::READUSERPROFILE"))
(unless (find-package "KORTEX_DRIVER::READUSERPROFILEREQUEST")
  (make-package "KORTEX_DRIVER::READUSERPROFILEREQUEST"))
(unless (find-package "KORTEX_DRIVER::READUSERPROFILERESPONSE")
  (make-package "KORTEX_DRIVER::READUSERPROFILERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadUserProfileRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadUserProfileRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::UserProfileHandle :init))
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
    ;; kortex_driver/UserProfileHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfileHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfileHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadUserProfileResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadUserProfileResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::UserProfile :init))
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
    ;; kortex_driver/UserProfile _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfile _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfile _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadUserProfile
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadUserProfile :md5sum-) "88ed6f69b3ce05a4b3c054859c8deeff")
(setf (get kortex_driver::ReadUserProfile :datatype-) "kortex_driver/ReadUserProfile")
(setf (get kortex_driver::ReadUserProfile :request) kortex_driver::ReadUserProfileRequest)
(setf (get kortex_driver::ReadUserProfile :response) kortex_driver::ReadUserProfileResponse)

(defmethod kortex_driver::ReadUserProfileRequest
  (:response () (instance kortex_driver::ReadUserProfileResponse :init)))

(setf (get kortex_driver::ReadUserProfileRequest :md5sum-) "88ed6f69b3ce05a4b3c054859c8deeff")
(setf (get kortex_driver::ReadUserProfileRequest :datatype-) "kortex_driver/ReadUserProfileRequest")
(setf (get kortex_driver::ReadUserProfileRequest :definition-)
      "UserProfileHandle input

================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
---
UserProfile output

================================================================================
MSG: kortex_driver/UserProfile

UserProfileHandle handle
string username
string firstname
string lastname
string application_data
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")

(setf (get kortex_driver::ReadUserProfileResponse :md5sum-) "88ed6f69b3ce05a4b3c054859c8deeff")
(setf (get kortex_driver::ReadUserProfileResponse :datatype-) "kortex_driver/ReadUserProfileResponse")
(setf (get kortex_driver::ReadUserProfileResponse :definition-)
      "UserProfileHandle input

================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
---
UserProfile output

================================================================================
MSG: kortex_driver/UserProfile

UserProfileHandle handle
string username
string firstname
string lastname
string application_data
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")



(provide :kortex_driver/ReadUserProfile "88ed6f69b3ce05a4b3c054859c8deeff")


