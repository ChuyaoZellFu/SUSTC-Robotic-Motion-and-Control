;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UserList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UserList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::USERLIST")
  (make-package "KORTEX_DRIVER::USERLIST"))

(in-package "ROS")
;;//! \htmlinclude UserList.msg.html


(defclass kortex_driver::UserList
  :super ros::object
  :slots (_user_handles ))

(defmethod kortex_driver::UserList
  (:init
   (&key
    ((:user_handles __user_handles) ())
    )
   (send-super :init)
   (setq _user_handles __user_handles)
   self)
  (:user_handles
   (&rest __user_handles)
   (if (keywordp (car __user_handles))
       (send* _user_handles __user_handles)
     (progn
       (if __user_handles (setq _user_handles (car __user_handles)))
       _user_handles)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/UserProfileHandle[] _user_handles
    (apply #'+ (send-all _user_handles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfileHandle[] _user_handles
     (write-long (length _user_handles) s)
     (dolist (elem _user_handles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfileHandle[] _user_handles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _user_handles (let (r) (dotimes (i n) (push (instance kortex_driver::UserProfileHandle :init) r)) r))
     (dolist (elem- _user_handles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::UserList :md5sum-) "fd1c8a5c2edbaa168a73081302b5bac5")
(setf (get kortex_driver::UserList :datatype-) "kortex_driver/UserList")
(setf (get kortex_driver::UserList :definition-)
      "
UserProfileHandle[] user_handles
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/UserList "fd1c8a5c2edbaa168a73081302b5bac5")


