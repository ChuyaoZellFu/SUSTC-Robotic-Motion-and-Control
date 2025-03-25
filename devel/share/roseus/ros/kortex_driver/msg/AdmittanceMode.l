;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::AdmittanceMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'AdmittanceMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ADMITTANCEMODE")
  (make-package "KORTEX_DRIVER::ADMITTANCEMODE"))

(in-package "ROS")
;;//! \htmlinclude AdmittanceMode.msg.html


(intern "*UNSPECIFIED_ADMITTANCE_MODE*" (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(shadow '*UNSPECIFIED_ADMITTANCE_MODE* (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(defconstant kortex_driver::AdmittanceMode::*UNSPECIFIED_ADMITTANCE_MODE* 0)
(intern "*CARTESIAN*" (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(shadow '*CARTESIAN* (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(defconstant kortex_driver::AdmittanceMode::*CARTESIAN* 1)
(intern "*JOINT*" (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(shadow '*JOINT* (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(defconstant kortex_driver::AdmittanceMode::*JOINT* 2)
(intern "*NULL_SPACE*" (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(shadow '*NULL_SPACE* (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(defconstant kortex_driver::AdmittanceMode::*NULL_SPACE* 3)
(intern "*DISABLED*" (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(shadow '*DISABLED* (find-package "KORTEX_DRIVER::ADMITTANCEMODE"))
(defconstant kortex_driver::AdmittanceMode::*DISABLED* 4)

(defun kortex_driver::AdmittanceMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::AdmittanceMode::*UNSPECIFIED_ADMITTANCE_MODE*)
        ((= const 1) 'kortex_driver::AdmittanceMode::*CARTESIAN*)
        ((= const 2) 'kortex_driver::AdmittanceMode::*JOINT*)
        ((= const 3) 'kortex_driver::AdmittanceMode::*NULL_SPACE*)
        ((= const 4) 'kortex_driver::AdmittanceMode::*DISABLED*)
        (t nil)))

(defclass kortex_driver::AdmittanceMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::AdmittanceMode
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

(setf (get kortex_driver::AdmittanceMode :md5sum-) "1a52f2d7eb92e509117483a6b9c79ce1")
(setf (get kortex_driver::AdmittanceMode :datatype-) "kortex_driver/AdmittanceMode")
(setf (get kortex_driver::AdmittanceMode :definition-)
      "
uint32 UNSPECIFIED_ADMITTANCE_MODE = 0

uint32 CARTESIAN = 1

uint32 JOINT = 2

uint32 NULL_SPACE = 3

uint32 DISABLED = 4

")



(provide :kortex_driver/AdmittanceMode "1a52f2d7eb92e509117483a6b9c79ce1")


