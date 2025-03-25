;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BackupEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BackupEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BACKUPEVENT")
  (make-package "KORTEX_DRIVER::BACKUPEVENT"))

(in-package "ROS")
;;//! \htmlinclude BackupEvent.msg.html


(intern "*UNSPECIFIED_BACKUP_EVENT*" (find-package "KORTEX_DRIVER::BACKUPEVENT"))
(shadow '*UNSPECIFIED_BACKUP_EVENT* (find-package "KORTEX_DRIVER::BACKUPEVENT"))
(defconstant kortex_driver::BackupEvent::*UNSPECIFIED_BACKUP_EVENT* 0)
(intern "*BACKUP_RESTORED*" (find-package "KORTEX_DRIVER::BACKUPEVENT"))
(shadow '*BACKUP_RESTORED* (find-package "KORTEX_DRIVER::BACKUPEVENT"))
(defconstant kortex_driver::BackupEvent::*BACKUP_RESTORED* 1)
(intern "*BACKUP_UPLOADED*" (find-package "KORTEX_DRIVER::BACKUPEVENT"))
(shadow '*BACKUP_UPLOADED* (find-package "KORTEX_DRIVER::BACKUPEVENT"))
(defconstant kortex_driver::BackupEvent::*BACKUP_UPLOADED* 2)

(defun kortex_driver::BackupEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::BackupEvent::*UNSPECIFIED_BACKUP_EVENT*)
        ((= const 1) 'kortex_driver::BackupEvent::*BACKUP_RESTORED*)
        ((= const 2) 'kortex_driver::BackupEvent::*BACKUP_UPLOADED*)
        (t nil)))

(defclass kortex_driver::BackupEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::BackupEvent
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

(setf (get kortex_driver::BackupEvent :md5sum-) "a96436414fbd5b9a597d2aa04b026d71")
(setf (get kortex_driver::BackupEvent :datatype-) "kortex_driver/BackupEvent")
(setf (get kortex_driver::BackupEvent :definition-)
      "
uint32 UNSPECIFIED_BACKUP_EVENT = 0

uint32 BACKUP_RESTORED = 1

uint32 BACKUP_UPLOADED = 2

")



(provide :kortex_driver/BackupEvent "a96436414fbd5b9a597d2aa04b026d71")


