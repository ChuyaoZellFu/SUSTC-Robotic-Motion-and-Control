;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CommandMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CommandMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COMMANDMODE")
  (make-package "KORTEX_DRIVER::COMMANDMODE"))

(in-package "ROS")
;;//! \htmlinclude CommandMode.msg.html


(intern "*CYCLIC*" (find-package "KORTEX_DRIVER::COMMANDMODE"))
(shadow '*CYCLIC* (find-package "KORTEX_DRIVER::COMMANDMODE"))
(defconstant kortex_driver::CommandMode::*CYCLIC* 0)
(intern "*ASYNC_CYCLIC_FLAGS*" (find-package "KORTEX_DRIVER::COMMANDMODE"))
(shadow '*ASYNC_CYCLIC_FLAGS* (find-package "KORTEX_DRIVER::COMMANDMODE"))
(defconstant kortex_driver::CommandMode::*ASYNC_CYCLIC_FLAGS* 1)
(intern "*ASYNC*" (find-package "KORTEX_DRIVER::COMMANDMODE"))
(shadow '*ASYNC* (find-package "KORTEX_DRIVER::COMMANDMODE"))
(defconstant kortex_driver::CommandMode::*ASYNC* 2)
(intern "*CYCLIC_JITTERCOMPENSATED_POSITION*" (find-package "KORTEX_DRIVER::COMMANDMODE"))
(shadow '*CYCLIC_JITTERCOMPENSATED_POSITION* (find-package "KORTEX_DRIVER::COMMANDMODE"))
(defconstant kortex_driver::CommandMode::*CYCLIC_JITTERCOMPENSATED_POSITION* 3)
(intern "*CYCLIC_JITTERCOMPENSATED_VELOCITY*" (find-package "KORTEX_DRIVER::COMMANDMODE"))
(shadow '*CYCLIC_JITTERCOMPENSATED_VELOCITY* (find-package "KORTEX_DRIVER::COMMANDMODE"))
(defconstant kortex_driver::CommandMode::*CYCLIC_JITTERCOMPENSATED_VELOCITY* 4)
(intern "*CYCLIC_JITTERCOMPENSATED_ACCELERATION*" (find-package "KORTEX_DRIVER::COMMANDMODE"))
(shadow '*CYCLIC_JITTERCOMPENSATED_ACCELERATION* (find-package "KORTEX_DRIVER::COMMANDMODE"))
(defconstant kortex_driver::CommandMode::*CYCLIC_JITTERCOMPENSATED_ACCELERATION* 5)

(defun kortex_driver::CommandMode-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::CommandMode::*CYCLIC*)
        ((= const 1) 'kortex_driver::CommandMode::*ASYNC_CYCLIC_FLAGS*)
        ((= const 2) 'kortex_driver::CommandMode::*ASYNC*)
        ((= const 3) 'kortex_driver::CommandMode::*CYCLIC_JITTERCOMPENSATED_POSITION*)
        ((= const 4) 'kortex_driver::CommandMode::*CYCLIC_JITTERCOMPENSATED_VELOCITY*)
        ((= const 5) 'kortex_driver::CommandMode::*CYCLIC_JITTERCOMPENSATED_ACCELERATION*)
        (t nil)))

(defclass kortex_driver::CommandMode
  :super ros::object
  :slots ())

(defmethod kortex_driver::CommandMode
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

(setf (get kortex_driver::CommandMode :md5sum-) "2d888c2b80a47e457086f44a5dd3543e")
(setf (get kortex_driver::CommandMode :datatype-) "kortex_driver/CommandMode")
(setf (get kortex_driver::CommandMode :definition-)
      "
uint32 CYCLIC = 0

uint32 ASYNC_CYCLIC_FLAGS = 1

uint32 ASYNC = 2

uint32 CYCLIC_JITTERCOMPENSATED_POSITION = 3

uint32 CYCLIC_JITTERCOMPENSATED_VELOCITY = 4

uint32 CYCLIC_JITTERCOMPENSATED_ACCELERATION = 5

")



(provide :kortex_driver/CommandMode "2d888c2b80a47e457086f44a5dd3543e")


