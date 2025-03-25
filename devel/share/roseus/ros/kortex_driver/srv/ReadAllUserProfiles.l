;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadAllUserProfiles)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadAllUserProfiles (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READALLUSERPROFILES")
  (make-package "KORTEX_DRIVER::READALLUSERPROFILES"))
(unless (find-package "KORTEX_DRIVER::READALLUSERPROFILESREQUEST")
  (make-package "KORTEX_DRIVER::READALLUSERPROFILESREQUEST"))
(unless (find-package "KORTEX_DRIVER::READALLUSERPROFILESRESPONSE")
  (make-package "KORTEX_DRIVER::READALLUSERPROFILESRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadAllUserProfilesRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadAllUserProfilesRequest
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

(defclass kortex_driver::ReadAllUserProfilesResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadAllUserProfilesResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::UserProfileList :init))
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
    ;; kortex_driver/UserProfileList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfileList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfileList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadAllUserProfiles
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadAllUserProfiles :md5sum-) "682c67f2598d4e520be0fc6015a67bc4")
(setf (get kortex_driver::ReadAllUserProfiles :datatype-) "kortex_driver/ReadAllUserProfiles")
(setf (get kortex_driver::ReadAllUserProfiles :request) kortex_driver::ReadAllUserProfilesRequest)
(setf (get kortex_driver::ReadAllUserProfiles :response) kortex_driver::ReadAllUserProfilesResponse)

(defmethod kortex_driver::ReadAllUserProfilesRequest
  (:response () (instance kortex_driver::ReadAllUserProfilesResponse :init)))

(setf (get kortex_driver::ReadAllUserProfilesRequest :md5sum-) "682c67f2598d4e520be0fc6015a67bc4")
(setf (get kortex_driver::ReadAllUserProfilesRequest :datatype-) "kortex_driver/ReadAllUserProfilesRequest")
(setf (get kortex_driver::ReadAllUserProfilesRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
UserProfileList output

================================================================================
MSG: kortex_driver/UserProfileList

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
")

(setf (get kortex_driver::ReadAllUserProfilesResponse :md5sum-) "682c67f2598d4e520be0fc6015a67bc4")
(setf (get kortex_driver::ReadAllUserProfilesResponse :datatype-) "kortex_driver/ReadAllUserProfilesResponse")
(setf (get kortex_driver::ReadAllUserProfilesResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
UserProfileList output

================================================================================
MSG: kortex_driver/UserProfileList

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
")



(provide :kortex_driver/ReadAllUserProfiles "682c67f2598d4e520be0fc6015a67bc4")


