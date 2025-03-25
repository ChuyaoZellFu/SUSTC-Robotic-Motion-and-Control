;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GPIOPull)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GPIOPull (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOPULL")
  (make-package "KORTEX_DRIVER::GPIOPULL"))

(in-package "ROS")
;;//! \htmlinclude GPIOPull.msg.html


(intern "*GPIO_PULL_UNSPECIFIED*" (find-package "KORTEX_DRIVER::GPIOPULL"))
(shadow '*GPIO_PULL_UNSPECIFIED* (find-package "KORTEX_DRIVER::GPIOPULL"))
(defconstant kortex_driver::GPIOPull::*GPIO_PULL_UNSPECIFIED* 0)
(intern "*GPIO_PULL_NONE*" (find-package "KORTEX_DRIVER::GPIOPULL"))
(shadow '*GPIO_PULL_NONE* (find-package "KORTEX_DRIVER::GPIOPULL"))
(defconstant kortex_driver::GPIOPull::*GPIO_PULL_NONE* 1)
(intern "*GPIO_PULL_UP*" (find-package "KORTEX_DRIVER::GPIOPULL"))
(shadow '*GPIO_PULL_UP* (find-package "KORTEX_DRIVER::GPIOPULL"))
(defconstant kortex_driver::GPIOPull::*GPIO_PULL_UP* 2)
(intern "*GPIO_PULL_DOWN*" (find-package "KORTEX_DRIVER::GPIOPULL"))
(shadow '*GPIO_PULL_DOWN* (find-package "KORTEX_DRIVER::GPIOPULL"))
(defconstant kortex_driver::GPIOPull::*GPIO_PULL_DOWN* 3)

(defun kortex_driver::GPIOPull-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GPIOPull::*GPIO_PULL_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::GPIOPull::*GPIO_PULL_NONE*)
        ((= const 2) 'kortex_driver::GPIOPull::*GPIO_PULL_UP*)
        ((= const 3) 'kortex_driver::GPIOPull::*GPIO_PULL_DOWN*)
        (t nil)))

(defclass kortex_driver::GPIOPull
  :super ros::object
  :slots ())

(defmethod kortex_driver::GPIOPull
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

(setf (get kortex_driver::GPIOPull :md5sum-) "76d74f4a9f22d9530bd83becdc56fba5")
(setf (get kortex_driver::GPIOPull :datatype-) "kortex_driver/GPIOPull")
(setf (get kortex_driver::GPIOPull :definition-)
      "
uint32 GPIO_PULL_UNSPECIFIED = 0

uint32 GPIO_PULL_NONE = 1

uint32 GPIO_PULL_UP = 2

uint32 GPIO_PULL_DOWN = 3

")



(provide :kortex_driver/GPIOPull "76d74f4a9f22d9530bd83becdc56fba5")


