;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EventIdSequenceInfoNotification)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EventIdSequenceInfoNotification (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION")
  (make-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))

(in-package "ROS")
;;//! \htmlinclude EventIdSequenceInfoNotification.msg.html


(intern "*UNSPECIFIED_SEQUENCE_EVENT*" (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(shadow '*UNSPECIFIED_SEQUENCE_EVENT* (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(defconstant kortex_driver::EventIdSequenceInfoNotification::*UNSPECIFIED_SEQUENCE_EVENT* 0)
(intern "*SEQUENCE_COMPLETED*" (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(shadow '*SEQUENCE_COMPLETED* (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(defconstant kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_COMPLETED* 1)
(intern "*SEQUENCE_ABORTED*" (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(shadow '*SEQUENCE_ABORTED* (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(defconstant kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_ABORTED* 2)
(intern "*SEQUENCE_PAUSED*" (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(shadow '*SEQUENCE_PAUSED* (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(defconstant kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_PAUSED* 3)
(intern "*SEQUENCE_TASK_STARTED*" (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(shadow '*SEQUENCE_TASK_STARTED* (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(defconstant kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_TASK_STARTED* 4)
(intern "*SEQUENCE_TASK_COMPLETED*" (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(shadow '*SEQUENCE_TASK_COMPLETED* (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(defconstant kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_TASK_COMPLETED* 5)
(intern "*SEQUENCE_STARTED*" (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(shadow '*SEQUENCE_STARTED* (find-package "KORTEX_DRIVER::EVENTIDSEQUENCEINFONOTIFICATION"))
(defconstant kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_STARTED* 6)

(defun kortex_driver::EventIdSequenceInfoNotification-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::EventIdSequenceInfoNotification::*UNSPECIFIED_SEQUENCE_EVENT*)
        ((= const 1) 'kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_COMPLETED*)
        ((= const 2) 'kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_ABORTED*)
        ((= const 3) 'kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_PAUSED*)
        ((= const 4) 'kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_TASK_STARTED*)
        ((= const 5) 'kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_TASK_COMPLETED*)
        ((= const 6) 'kortex_driver::EventIdSequenceInfoNotification::*SEQUENCE_STARTED*)
        (t nil)))

(defclass kortex_driver::EventIdSequenceInfoNotification
  :super ros::object
  :slots ())

(defmethod kortex_driver::EventIdSequenceInfoNotification
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

(setf (get kortex_driver::EventIdSequenceInfoNotification :md5sum-) "68fe86a38033c2b20696485a1800dc6a")
(setf (get kortex_driver::EventIdSequenceInfoNotification :datatype-) "kortex_driver/EventIdSequenceInfoNotification")
(setf (get kortex_driver::EventIdSequenceInfoNotification :definition-)
      "
uint32 UNSPECIFIED_SEQUENCE_EVENT = 0

uint32 SEQUENCE_COMPLETED = 1

uint32 SEQUENCE_ABORTED = 2

uint32 SEQUENCE_PAUSED = 3

uint32 SEQUENCE_TASK_STARTED = 4

uint32 SEQUENCE_TASK_COMPLETED = 5

uint32 SEQUENCE_STARTED = 6

")



(provide :kortex_driver/EventIdSequenceInfoNotification "68fe86a38033c2b20696485a1800dc6a")


