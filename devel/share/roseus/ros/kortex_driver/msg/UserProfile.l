;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UserProfile)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UserProfile (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::USERPROFILE")
  (make-package "KORTEX_DRIVER::USERPROFILE"))

(in-package "ROS")
;;//! \htmlinclude UserProfile.msg.html


(defclass kortex_driver::UserProfile
  :super ros::object
  :slots (_handle _username _firstname _lastname _application_data ))

(defmethod kortex_driver::UserProfile
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::UserProfileHandle :init))
    ((:username __username) "")
    ((:firstname __firstname) "")
    ((:lastname __lastname) "")
    ((:application_data __application_data) "")
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _username (string __username))
   (setq _firstname (string __firstname))
   (setq _lastname (string __lastname))
   (setq _application_data (string __application_data))
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:username
   (&optional __username)
   (if __username (setq _username __username)) _username)
  (:firstname
   (&optional __firstname)
   (if __firstname (setq _firstname __firstname)) _firstname)
  (:lastname
   (&optional __lastname)
   (if __lastname (setq _lastname __lastname)) _lastname)
  (:application_data
   (&optional __application_data)
   (if __application_data (setq _application_data __application_data)) _application_data)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/UserProfileHandle _handle
    (send _handle :serialization-length)
    ;; string _username
    4 (length _username)
    ;; string _firstname
    4 (length _firstname)
    ;; string _lastname
    4 (length _lastname)
    ;; string _application_data
    4 (length _application_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfileHandle _handle
       (send _handle :serialize s)
     ;; string _username
       (write-long (length _username) s) (princ _username s)
     ;; string _firstname
       (write-long (length _firstname) s) (princ _firstname s)
     ;; string _lastname
       (write-long (length _lastname) s) (princ _lastname s)
     ;; string _application_data
       (write-long (length _application_data) s) (princ _application_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfileHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; string _username
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _username (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _firstname
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _firstname (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lastname
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lastname (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _application_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _application_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::UserProfile :md5sum-) "4599942b2b24021432dda0e643774952")
(setf (get kortex_driver::UserProfile :datatype-) "kortex_driver/UserProfile")
(setf (get kortex_driver::UserProfile :definition-)
      "
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



(provide :kortex_driver/UserProfile "4599942b2b24021432dda0e643774952")


