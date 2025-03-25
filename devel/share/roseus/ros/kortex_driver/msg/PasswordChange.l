;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PasswordChange)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PasswordChange (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PASSWORDCHANGE")
  (make-package "KORTEX_DRIVER::PASSWORDCHANGE"))

(in-package "ROS")
;;//! \htmlinclude PasswordChange.msg.html


(defclass kortex_driver::PasswordChange
  :super ros::object
  :slots (_handle _old_password _new_password ))

(defmethod kortex_driver::PasswordChange
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::UserProfileHandle :init))
    ((:old_password __old_password) "")
    ((:new_password __new_password) "")
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _old_password (string __old_password))
   (setq _new_password (string __new_password))
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:old_password
   (&optional __old_password)
   (if __old_password (setq _old_password __old_password)) _old_password)
  (:new_password
   (&optional __new_password)
   (if __new_password (setq _new_password __new_password)) _new_password)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/UserProfileHandle _handle
    (send _handle :serialization-length)
    ;; string _old_password
    4 (length _old_password)
    ;; string _new_password
    4 (length _new_password)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/UserProfileHandle _handle
       (send _handle :serialize s)
     ;; string _old_password
       (write-long (length _old_password) s) (princ _old_password s)
     ;; string _new_password
       (write-long (length _new_password) s) (princ _new_password s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/UserProfileHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; string _old_password
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _old_password (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _new_password
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _new_password (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::PasswordChange :md5sum-) "75d13d1936094613c5e6bc629ee347b3")
(setf (get kortex_driver::PasswordChange :datatype-) "kortex_driver/PasswordChange")
(setf (get kortex_driver::PasswordChange :definition-)
      "
UserProfileHandle handle
string old_password
string new_password
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/PasswordChange "75d13d1936094613c5e6bc629ee347b3")


