;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FactoryEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FactoryEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FACTORYEVENT")
  (make-package "KORTEX_DRIVER::FACTORYEVENT"))

(in-package "ROS")
;;//! \htmlinclude FactoryEvent.msg.html


(intern "*UNSPECIFIED_FACTORY_EVENT*" (find-package "KORTEX_DRIVER::FACTORYEVENT"))
(shadow '*UNSPECIFIED_FACTORY_EVENT* (find-package "KORTEX_DRIVER::FACTORYEVENT"))
(defconstant kortex_driver::FactoryEvent::*UNSPECIFIED_FACTORY_EVENT* 0)
(intern "*FACTORY_DEFAULT_RESTORED*" (find-package "KORTEX_DRIVER::FACTORYEVENT"))
(shadow '*FACTORY_DEFAULT_RESTORED* (find-package "KORTEX_DRIVER::FACTORYEVENT"))
(defconstant kortex_driver::FactoryEvent::*FACTORY_DEFAULT_RESTORED* 1)
(intern "*NETWORK_FACTORY_DEFAULT_RESTORED*" (find-package "KORTEX_DRIVER::FACTORYEVENT"))
(shadow '*NETWORK_FACTORY_DEFAULT_RESTORED* (find-package "KORTEX_DRIVER::FACTORYEVENT"))
(defconstant kortex_driver::FactoryEvent::*NETWORK_FACTORY_DEFAULT_RESTORED* 2)

(defun kortex_driver::FactoryEvent-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::FactoryEvent::*UNSPECIFIED_FACTORY_EVENT*)
        ((= const 1) 'kortex_driver::FactoryEvent::*FACTORY_DEFAULT_RESTORED*)
        ((= const 2) 'kortex_driver::FactoryEvent::*NETWORK_FACTORY_DEFAULT_RESTORED*)
        (t nil)))

(defclass kortex_driver::FactoryEvent
  :super ros::object
  :slots ())

(defmethod kortex_driver::FactoryEvent
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

(setf (get kortex_driver::FactoryEvent :md5sum-) "f21e922380f9063597fbdefcc7ac55fa")
(setf (get kortex_driver::FactoryEvent :datatype-) "kortex_driver/FactoryEvent")
(setf (get kortex_driver::FactoryEvent :definition-)
      "
uint32 UNSPECIFIED_FACTORY_EVENT = 0

uint32 FACTORY_DEFAULT_RESTORED = 1

uint32 NETWORK_FACTORY_DEFAULT_RESTORED = 2

")



(provide :kortex_driver/FactoryEvent "f21e922380f9063597fbdefcc7ac55fa")


