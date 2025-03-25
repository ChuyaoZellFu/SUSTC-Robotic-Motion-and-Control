;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Permission)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Permission (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PERMISSION")
  (make-package "KORTEX_DRIVER::PERMISSION"))

(in-package "ROS")
;;//! \htmlinclude Permission.msg.html


(intern "*NO_PERMISSION*" (find-package "KORTEX_DRIVER::PERMISSION"))
(shadow '*NO_PERMISSION* (find-package "KORTEX_DRIVER::PERMISSION"))
(defconstant kortex_driver::Permission::*NO_PERMISSION* 0)
(intern "*READ_PERMISSION*" (find-package "KORTEX_DRIVER::PERMISSION"))
(shadow '*READ_PERMISSION* (find-package "KORTEX_DRIVER::PERMISSION"))
(defconstant kortex_driver::Permission::*READ_PERMISSION* 1)
(intern "*UPDATE_PERMISSION*" (find-package "KORTEX_DRIVER::PERMISSION"))
(shadow '*UPDATE_PERMISSION* (find-package "KORTEX_DRIVER::PERMISSION"))
(defconstant kortex_driver::Permission::*UPDATE_PERMISSION* 2)
(intern "*DELETE_PERMISSION*" (find-package "KORTEX_DRIVER::PERMISSION"))
(shadow '*DELETE_PERMISSION* (find-package "KORTEX_DRIVER::PERMISSION"))
(defconstant kortex_driver::Permission::*DELETE_PERMISSION* 4)

(defun kortex_driver::Permission-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::Permission::*NO_PERMISSION*)
        ((= const 1) 'kortex_driver::Permission::*READ_PERMISSION*)
        ((= const 2) 'kortex_driver::Permission::*UPDATE_PERMISSION*)
        ((= const 4) 'kortex_driver::Permission::*DELETE_PERMISSION*)
        (t nil)))

(defclass kortex_driver::Permission
  :super ros::object
  :slots ())

(defmethod kortex_driver::Permission
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::Permission :md5sum-) "2ae002b57072e72c8a5ded36bcd8b246")
(setf (get kortex_driver::Permission :datatype-) "kortex_driver/Permission")
(setf (get kortex_driver::Permission :definition-)
      "
uint32 NO_PERMISSION = 0

uint32 READ_PERMISSION = 1

uint32 UPDATE_PERMISSION = 2

uint32 DELETE_PERMISSION = 4

")



(provide :kortex_driver/Permission "2ae002b57072e72c8a5ded36bcd8b246")


