;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FullUserProfile)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FullUserProfile (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FULLUSERPROFILE")
  (make-package "KORTEX_DRIVER::FULLUSERPROFILE"))

(in-package "ROS")
;;//! \htmlinclude FullUserProfile.msg.html


(defclass kortex_driver::FullUserProfile
  :super ros::object
  :slots (_user_profile _password ))

(defmethod kortex_driver::FullUserProfile
  (:init
   (&key
    ((:user_profile __user_profile) (instance kortex_driver::UserProfile :init))
    ((:password __password) "")
    )
   (send-super :init)
   (setq _user_profile __user_profile)
   (setq _password (string __password))
   self)
  (:user_profile
   (&rest __user_profile)
   (if (keywordp (car __user_profile))
       (send* _user_profile __user_profile)
     (progn
       (if __user_profile (setq _user_profile (car __user_profile)))
       _user_profile)))
  (:password
   (&optional __password)
   (if __password (setq _password __password)) _password)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/UserProfile _user_profile
    (send _user_profile :serialization-length)
    ;; string _password
    4 (length _password)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfile _user_profile
       (send _user_profile :serialize s)
     ;; string _password
       (write-long (length _password) s) (princ _password s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfile _user_profile
     (send _user_profile :deserialize buf ptr-) (incf ptr- (send _user_profile :serialization-length))
   ;; string _password
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _password (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::FullUserProfile :md5sum-) "5832f6aa4b0c784ce33a85ff505da582")
(setf (get kortex_driver::FullUserProfile :datatype-) "kortex_driver/FullUserProfile")
(setf (get kortex_driver::FullUserProfile :definition-)
      "
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
uint32 permission
")



(provide :kortex_driver/FullUserProfile "5832f6aa4b0c784ce33a85ff505da582")


