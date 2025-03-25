;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectCyclic_ServiceVersion)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectCyclic_ServiceVersion (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_SERVICEVERSION")
  (make-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_SERVICEVERSION"))

(in-package "ROS")
;;//! \htmlinclude InterconnectCyclic_ServiceVersion.msg.html


(intern "*RESERVED_0*" (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_SERVICEVERSION"))
(shadow '*RESERVED_0* (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_SERVICEVERSION"))
(defconstant kortex_driver::InterconnectCyclic_ServiceVersion::*RESERVED_0* 0)
(intern "*CURRENT_VERSION*" (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_SERVICEVERSION"))
(shadow '*CURRENT_VERSION* (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_SERVICEVERSION"))
(defconstant kortex_driver::InterconnectCyclic_ServiceVersion::*CURRENT_VERSION* 1)

(defun kortex_driver::InterconnectCyclic_ServiceVersion-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::InterconnectCyclic_ServiceVersion::*RESERVED_0*)
        ((= const 1) 'kortex_driver::InterconnectCyclic_ServiceVersion::*CURRENT_VERSION*)
        (t nil)))

(defclass kortex_driver::InterconnectCyclic_ServiceVersion
  :super ros::object
  :slots ())

(defmethod kortex_driver::InterconnectCyclic_ServiceVersion
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

(setf (get kortex_driver::InterconnectCyclic_ServiceVersion :md5sum-) "d8624be8e7830df48d9964b7612fa5b3")
(setf (get kortex_driver::InterconnectCyclic_ServiceVersion :datatype-) "kortex_driver/InterconnectCyclic_ServiceVersion")
(setf (get kortex_driver::InterconnectCyclic_ServiceVersion :definition-)
      "
uint32 RESERVED_0 = 0

uint32 CURRENT_VERSION = 1

")



(provide :kortex_driver/InterconnectCyclic_ServiceVersion "d8624be8e7830df48d9964b7612fa5b3")


