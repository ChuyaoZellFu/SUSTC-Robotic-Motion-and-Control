;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::NavigationDirection)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'NavigationDirection (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION")
  (make-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))

(in-package "ROS")
;;//! \htmlinclude NavigationDirection.msg.html


(intern "*UNSPECIFIED_NAVIGATION_DIRECTION*" (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(shadow '*UNSPECIFIED_NAVIGATION_DIRECTION* (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(defconstant kortex_driver::NavigationDirection::*UNSPECIFIED_NAVIGATION_DIRECTION* 0)
(intern "*NEXT*" (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(shadow '*NEXT* (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(defconstant kortex_driver::NavigationDirection::*NEXT* 1)
(intern "*UP*" (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(shadow '*UP* (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(defconstant kortex_driver::NavigationDirection::*UP* 2)
(intern "*DOWN*" (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(shadow '*DOWN* (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(defconstant kortex_driver::NavigationDirection::*DOWN* 3)
(intern "*PREVIOUS*" (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(shadow '*PREVIOUS* (find-package "KORTEX_DRIVER::NAVIGATIONDIRECTION"))
(defconstant kortex_driver::NavigationDirection::*PREVIOUS* 4)

(defun kortex_driver::NavigationDirection-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::NavigationDirection::*UNSPECIFIED_NAVIGATION_DIRECTION*)
        ((= const 1) 'kortex_driver::NavigationDirection::*NEXT*)
        ((= const 2) 'kortex_driver::NavigationDirection::*UP*)
        ((= const 3) 'kortex_driver::NavigationDirection::*DOWN*)
        ((= const 4) 'kortex_driver::NavigationDirection::*PREVIOUS*)
        (t nil)))

(defclass kortex_driver::NavigationDirection
  :super ros::object
  :slots ())

(defmethod kortex_driver::NavigationDirection
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

(setf (get kortex_driver::NavigationDirection :md5sum-) "db0163da7423639bdc46c2cceb491c80")
(setf (get kortex_driver::NavigationDirection :datatype-) "kortex_driver/NavigationDirection")
(setf (get kortex_driver::NavigationDirection :definition-)
      "
uint32 UNSPECIFIED_NAVIGATION_DIRECTION = 0

uint32 NEXT = 1

uint32 UP = 2

uint32 DOWN = 3

uint32 PREVIOUS = 4

")



(provide :kortex_driver/NavigationDirection "db0163da7423639bdc46c2cceb491c80")


