;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UpdateUserProfile)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UpdateUserProfile (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UPDATEUSERPROFILE")
  (make-package "KORTEX_DRIVER::UPDATEUSERPROFILE"))
(unless (find-package "KORTEX_DRIVER::UPDATEUSERPROFILEREQUEST")
  (make-package "KORTEX_DRIVER::UPDATEUSERPROFILEREQUEST"))
(unless (find-package "KORTEX_DRIVER::UPDATEUSERPROFILERESPONSE")
  (make-package "KORTEX_DRIVER::UPDATEUSERPROFILERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::UpdateUserProfileRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::UpdateUserProfileRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::UserProfile :init))
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
    ;; kortex_driver/UserProfile _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfile _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfile _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::UpdateUserProfileResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::UpdateUserProfileResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::UpdateUserProfile
  :super ros::object
  :slots ())

(setf (get kortex_driver::UpdateUserProfile :md5sum-) "75eb07981bad53b9f3b10bd6121afb95")
(setf (get kortex_driver::UpdateUserProfile :datatype-) "kortex_driver/UpdateUserProfile")
(setf (get kortex_driver::UpdateUserProfile :request) kortex_driver::UpdateUserProfileRequest)
(setf (get kortex_driver::UpdateUserProfile :response) kortex_driver::UpdateUserProfileResponse)

(defmethod kortex_driver::UpdateUserProfileRequest
  (:response () (instance kortex_driver::UpdateUserProfileResponse :init)))

(setf (get kortex_driver::UpdateUserProfileRequest :md5sum-) "75eb07981bad53b9f3b10bd6121afb95")
(setf (get kortex_driver::UpdateUserProfileRequest :datatype-) "kortex_driver/UpdateUserProfileRequest")
(setf (get kortex_driver::UpdateUserProfileRequest :definition-)
      "UserProfile input

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
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::UpdateUserProfileResponse :md5sum-) "75eb07981bad53b9f3b10bd6121afb95")
(setf (get kortex_driver::UpdateUserProfileResponse :datatype-) "kortex_driver/UpdateUserProfileResponse")
(setf (get kortex_driver::UpdateUserProfileResponse :definition-)
      "UserProfile input

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
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/UpdateUserProfile "75eb07981bad53b9f3b10bd6121afb95")


