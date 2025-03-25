;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointNavigationDirection)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointNavigationDirection (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION")
  (make-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION"))

(in-package "ROS")
;;//! \htmlinclude JointNavigationDirection.msg.html


(intern "*UNSPECIFIED_JOINT_NAVIGATION_DIRECTION*" (find-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION"))
(shadow '*UNSPECIFIED_JOINT_NAVIGATION_DIRECTION* (find-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION"))
(defconstant kortex_driver::JointNavigationDirection::*UNSPECIFIED_JOINT_NAVIGATION_DIRECTION* 0)
(intern "*JOINT_NEXT*" (find-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION"))
(shadow '*JOINT_NEXT* (find-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION"))
(defconstant kortex_driver::JointNavigationDirection::*JOINT_NEXT* 1)
(intern "*JOINT_PREVIOUS*" (find-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION"))
(shadow '*JOINT_PREVIOUS* (find-package "KORTEX_DRIVER::JOINTNAVIGATIONDIRECTION"))
(defconstant kortex_driver::JointNavigationDirection::*JOINT_PREVIOUS* 2)

(defun kortex_driver::JointNavigationDirection-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::JointNavigationDirection::*UNSPECIFIED_JOINT_NAVIGATION_DIRECTION*)
        ((= const 1) 'kortex_driver::JointNavigationDirection::*JOINT_NEXT*)
        ((= const 2) 'kortex_driver::JointNavigationDirection::*JOINT_PREVIOUS*)
        (t nil)))

(defclass kortex_driver::JointNavigationDirection
  :super ros::object
  :slots ())

(defmethod kortex_driver::JointNavigationDirection
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

(setf (get kortex_driver::JointNavigationDirection :md5sum-) "718d5eb2fe311f7d85c6686a0a7a3c9a")
(setf (get kortex_driver::JointNavigationDirection :datatype-) "kortex_driver/JointNavigationDirection")
(setf (get kortex_driver::JointNavigationDirection :definition-)
      "
uint32 UNSPECIFIED_JOINT_NAVIGATION_DIRECTION = 0

uint32 JOINT_NEXT = 1

uint32 JOINT_PREVIOUS = 2

")



(provide :kortex_driver/JointNavigationDirection "718d5eb2fe311f7d85c6686a0a7a3c9a")


