;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CreateUserProfile)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CreateUserProfile (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CREATEUSERPROFILE")
  (make-package "KORTEX_DRIVER::CREATEUSERPROFILE"))
(unless (find-package "KORTEX_DRIVER::CREATEUSERPROFILEREQUEST")
  (make-package "KORTEX_DRIVER::CREATEUSERPROFILEREQUEST"))
(unless (find-package "KORTEX_DRIVER::CREATEUSERPROFILERESPONSE")
  (make-package "KORTEX_DRIVER::CREATEUSERPROFILERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::CreateUserProfileRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::CreateUserProfileRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::FullUserProfile :init))
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
    ;; kortex_driver/FullUserProfile _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/FullUserProfile _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/FullUserProfile _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::CreateUserProfileResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::CreateUserProfileResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::UserProfileHandle :init))
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
    ;; kortex_driver/UserProfileHandle _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfileHandle _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfileHandle _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::CreateUserProfile
  :super ros::object
  :slots ())

(setf (get kortex_driver::CreateUserProfile :md5sum-) "406efcad037e6ae1bf367e1fe039a21c")
(setf (get kortex_driver::CreateUserProfile :datatype-) "kortex_driver/CreateUserProfile")
(setf (get kortex_driver::CreateUserProfile :request) kortex_driver::CreateUserProfileRequest)
(setf (get kortex_driver::CreateUserProfile :response) kortex_driver::CreateUserProfileResponse)

(defmethod kortex_driver::CreateUserProfileRequest
  (:response () (instance kortex_driver::CreateUserProfileResponse :init)))

(setf (get kortex_driver::CreateUserProfileRequest :md5sum-) "406efcad037e6ae1bf367e1fe039a21c")
(setf (get kortex_driver::CreateUserProfileRequest :datatype-) "kortex_driver/CreateUserProfileRequest")
(setf (get kortex_driver::CreateUserProfileRequest :definition-)
      "FullUserProfile input

================================================================================
MSG: kortex_driver/FullUserProfile

UserProfile user_profile
string password
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
---
UserProfileHandle output

================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")

(setf (get kortex_driver::CreateUserProfileResponse :md5sum-) "406efcad037e6ae1bf367e1fe039a21c")
(setf (get kortex_driver::CreateUserProfileResponse :datatype-) "kortex_driver/CreateUserProfileResponse")
(setf (get kortex_driver::CreateUserProfileResponse :definition-)
      "FullUserProfile input

================================================================================
MSG: kortex_driver/FullUserProfile

UserProfile user_profile
string password
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
---
UserProfileHandle output

================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
")



(provide :kortex_driver/CreateUserProfile "406efcad037e6ae1bf367e1fe039a21c")


