;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EndEffectorType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EndEffectorType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE")
  (make-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))

(in-package "ROS")
;;//! \htmlinclude EndEffectorType.msg.html


(intern "*END_EFFECTOR_TYPE_UNSPECIFIED*" (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(shadow '*END_EFFECTOR_TYPE_UNSPECIFIED* (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(defconstant kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_UNSPECIFIED* 0)
(intern "*END_EFFECTOR_TYPE_NOT_INSTALLED*" (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(shadow '*END_EFFECTOR_TYPE_NOT_INSTALLED* (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(defconstant kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_NOT_INSTALLED* 1)
(intern "*END_EFFECTOR_TYPE_L31_GRIPPER_2F*" (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(shadow '*END_EFFECTOR_TYPE_L31_GRIPPER_2F* (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(defconstant kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_L31_GRIPPER_2F* 2)
(intern "*END_EFFECTOR_TYPE_ROBOTIQ_2F_85*" (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(shadow '*END_EFFECTOR_TYPE_ROBOTIQ_2F_85* (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(defconstant kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_ROBOTIQ_2F_85* 3)
(intern "*END_EFFECTOR_TYPE_ROBOTIQ_2F_140*" (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(shadow '*END_EFFECTOR_TYPE_ROBOTIQ_2F_140* (find-package "KORTEX_DRIVER::ENDEFFECTORTYPE"))
(defconstant kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_ROBOTIQ_2F_140* 4)

(defun kortex_driver::EndEffectorType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_NOT_INSTALLED*)
        ((= const 2) 'kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_L31_GRIPPER_2F*)
        ((= const 3) 'kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_ROBOTIQ_2F_85*)
        ((= const 4) 'kortex_driver::EndEffectorType::*END_EFFECTOR_TYPE_ROBOTIQ_2F_140*)
        (t nil)))

(defclass kortex_driver::EndEffectorType
  :super ros::object
  :slots ())

(defmethod kortex_driver::EndEffectorType
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

(setf (get kortex_driver::EndEffectorType :md5sum-) "93f96d9a78bf91aec505afd56097a885")
(setf (get kortex_driver::EndEffectorType :datatype-) "kortex_driver/EndEffectorType")
(setf (get kortex_driver::EndEffectorType :definition-)
      "
uint32 END_EFFECTOR_TYPE_UNSPECIFIED = 0

uint32 END_EFFECTOR_TYPE_NOT_INSTALLED = 1

uint32 END_EFFECTOR_TYPE_L31_GRIPPER_2F = 2

uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_85 = 3

uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_140 = 4

")



(provide :kortex_driver/EndEffectorType "93f96d9a78bf91aec505afd56097a885")


