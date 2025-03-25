;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UserProfileList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UserProfileList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::USERPROFILELIST")
  (make-package "KORTEX_DRIVER::USERPROFILELIST"))

(in-package "ROS")
;;//! \htmlinclude UserProfileList.msg.html


(defclass kortex_driver::UserProfileList
  :super ros::object
  :slots (_user_profiles ))

(defmethod kortex_driver::UserProfileList
  (:init
   (&key
    ((:user_profiles __user_profiles) ())
    )
   (send-super :init)
   (setq _user_profiles __user_profiles)
   self)
  (:user_profiles
   (&rest __user_profiles)
   (if (keywordp (car __user_profiles))
       (send* _user_profiles __user_profiles)
     (progn
       (if __user_profiles (setq _user_profiles (car __user_profiles)))
       _user_profiles)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/UserProfile[] _user_profiles
    (apply #'+ (send-all _user_profiles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfile[] _user_profiles
     (write-long (length _user_profiles) s)
     (dolist (elem _user_profiles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfile[] _user_profiles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _user_profiles (let (r) (dotimes (i n) (push (instance kortex_driver::UserProfile :init) r)) r))
     (dolist (elem- _user_profiles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::UserProfileList :md5sum-) "2d71d5ab953fb8ddd6c9d9d4a1379bb4")
(setf (get kortex_driver::UserProfileList :datatype-) "kortex_driver/UserProfileList")
(setf (get kortex_driver::UserProfileList :definition-)
      "
UserProfile[] user_profiles
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



(provide :kortex_driver/UserProfileList "2d71d5ab953fb8ddd6c9d9d4a1379bb4")


