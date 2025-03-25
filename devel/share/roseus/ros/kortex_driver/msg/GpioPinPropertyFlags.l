;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GpioPinPropertyFlags)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GpioPinPropertyFlags (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS")
  (make-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))

(in-package "ROS")
;;//! \htmlinclude GpioPinPropertyFlags.msg.html


(intern "*GPIOPROPERTY_UNKNOWN*" (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(shadow '*GPIOPROPERTY_UNKNOWN* (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(defconstant kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_UNKNOWN* 0)
(intern "*GPIOPROPERTY_INPUT*" (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(shadow '*GPIOPROPERTY_INPUT* (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(defconstant kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_INPUT* 1)
(intern "*GPIOPROPERTY_OUTPUT*" (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(shadow '*GPIOPROPERTY_OUTPUT* (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(defconstant kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_OUTPUT* 2)
(intern "*GPIOPROPERTY_ANALOG*" (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(shadow '*GPIOPROPERTY_ANALOG* (find-package "KORTEX_DRIVER::GPIOPINPROPERTYFLAGS"))
(defconstant kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_ANALOG* 4)

(defun kortex_driver::GpioPinPropertyFlags-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_UNKNOWN*)
        ((= const 1) 'kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_INPUT*)
        ((= const 2) 'kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_OUTPUT*)
        ((= const 4) 'kortex_driver::GpioPinPropertyFlags::*GPIOPROPERTY_ANALOG*)
        (t nil)))

(defclass kortex_driver::GpioPinPropertyFlags
  :super ros::object
  :slots ())

(defmethod kortex_driver::GpioPinPropertyFlags
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

(setf (get kortex_driver::GpioPinPropertyFlags :md5sum-) "531958ae411036543a3b84e9b7f802d3")
(setf (get kortex_driver::GpioPinPropertyFlags :datatype-) "kortex_driver/GpioPinPropertyFlags")
(setf (get kortex_driver::GpioPinPropertyFlags :definition-)
      "
uint32 GPIOPROPERTY_UNKNOWN = 0

uint32 GPIOPROPERTY_INPUT = 1

uint32 GPIOPROPERTY_OUTPUT = 2

uint32 GPIOPROPERTY_ANALOG = 4

")



(provide :kortex_driver/GpioPinPropertyFlags "531958ae411036543a3b84e9b7f802d3")


