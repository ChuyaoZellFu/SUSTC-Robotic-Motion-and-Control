;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WrenchMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WrenchMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WRENCHMODE")
  (make-package "KORTEX_DRIVER::WRENCHMODE"))

(in-package "ROS")
;;//! \htmlinclude WrenchMode.msg.html


(intern "*UNSPECIFIED_WRENCH_MODE*" (find-package "KORTEX_DRIVER::WRENCHMODE"))
(shadow '*UNSPECIFIED_WRENCH_MODE* (find-package "KORTEX_DRIVER::WRENCHMODE"))
(defconstant kortex_driver::WrenchMode::*UNSPECIFIED_WRENCH_MODE* 0)
(intern "*WRENCH_RESTRICTED*" (find-package "KORTEX_DRIVER::WRENCHMODE"))
(shadow '*WRENCH_RESTRICTED* (find-package "KORTEX_DRIVER::WRENCHMODE"))
(defconstant kortex_driver::WrenchMode::*WRENCH_RESTRICTED* 1)
(intern "*WRENCH_NORMAL*" (find-package "KORTEX_DRIVER::WRENCHMODE"))
(shadow '*WRENCH_NORMAL* (find-package "KORTEX_DRIVER::WRENCHMODE"))
(defconstant kortex_driver::WrenchMode::*WRENCH_NORMAL* 2)

(defun kortex_driver::WrenchMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::WrenchMode::*UNSPECIFIED_WRENCH_MODE*)
        ((= const 1) 'kortex_driver::WrenchMode::*WRENCH_RESTRICTED*)
        ((= const 2) 'kortex_driver::WrenchMode::*WRENCH_NORMAL*)
        (t nil)))

(defclass kortex_driver::WrenchMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::WrenchMode
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

(setf (get kortex_driver::WrenchMode :md5sum-) "21f4098a76f681f49eb3f39d12684c72")
(setf (get kortex_driver::WrenchMode :datatype-) "kortex_driver/WrenchMode")
(setf (get kortex_driver::WrenchMode :definition-)
      "
uint32 UNSPECIFIED_WRENCH_MODE = 0

uint32 WRENCH_RESTRICTED = 1

uint32 WRENCH_NORMAL = 2

")



(provide :kortex_driver/WrenchMode "21f4098a76f681f49eb3f39d12684c72")


