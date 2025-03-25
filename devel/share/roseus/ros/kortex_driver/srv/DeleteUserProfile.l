;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteUserProfile)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteUserProfile (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETEUSERPROFILE")
  (make-package "KORTEX_DRIVER::DELETEUSERPROFILE"))
(unless (find-package "KORTEX_DRIVER::DELETEUSERPROFILEREQUEST")
  (make-package "KORTEX_DRIVER::DELETEUSERPROFILEREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETEUSERPROFILERESPONSE")
  (make-package "KORTEX_DRIVER::DELETEUSERPROFILERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteUserProfileRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteUserProfileRequest
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

(defclass kortex_driver::DeleteUserProfileResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteUserProfileResponse
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

(defclass kortex_driver::DeleteUserProfile
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteUserProfile :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteUserProfile :datatype-) "kortex_driver/DeleteUserProfile")
(setf (get kortex_driver::DeleteUserProfile :request) kortex_driver::DeleteUserProfileRequest)
(setf (get kortex_driver::DeleteUserProfile :response) kortex_driver::DeleteUserProfileResponse)

(defmethod kortex_driver::DeleteUserProfileRequest
  (:response () (instance kortex_driver::DeleteUserProfileResponse :init)))

(setf (get kortex_driver::DeleteUserProfileRequest :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteUserProfileRequest :datatype-) "kortex_driver/DeleteUserProfileRequest")
(setf (get kortex_driver::DeleteUserProfileRequest :definition-)
      "UserProfileHandle input

================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteUserProfileResponse :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteUserProfileResponse :datatype-) "kortex_driver/DeleteUserProfileResponse")
(setf (get kortex_driver::DeleteUserProfileResponse :definition-)
      "UserProfileHandle input

================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteUserProfile "04dfaeca45772f660e0913aa84774e13")


