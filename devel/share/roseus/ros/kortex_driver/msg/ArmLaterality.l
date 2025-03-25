;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ArmLaterality)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ArmLaterality (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ARMLATERALITY")
  (make-package "KORTEX_DRIVER::ARMLATERALITY"))

(in-package "ROS")
;;//! \htmlinclude ArmLaterality.msg.html


(intern "*ARM_LATERALITY_UNSPECIFIED*" (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(shadow '*ARM_LATERALITY_UNSPECIFIED* (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(defconstant kortex_driver::ArmLaterality::*ARM_LATERALITY_UNSPECIFIED* 0)
(intern "*ARM_LATERALITY_NOT_APPLICABLE*" (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(shadow '*ARM_LATERALITY_NOT_APPLICABLE* (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(defconstant kortex_driver::ArmLaterality::*ARM_LATERALITY_NOT_APPLICABLE* 1)
(intern "*ARM_LATERALITY_LEFT*" (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(shadow '*ARM_LATERALITY_LEFT* (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(defconstant kortex_driver::ArmLaterality::*ARM_LATERALITY_LEFT* 2)
(intern "*ARM_LATERALITY_RIGHT*" (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(shadow '*ARM_LATERALITY_RIGHT* (find-package "KORTEX_DRIVER::ARMLATERALITY"))
(defconstant kortex_driver::ArmLaterality::*ARM_LATERALITY_RIGHT* 3)

(defun kortex_driver::ArmLaterality-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ArmLaterality::*ARM_LATERALITY_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::ArmLaterality::*ARM_LATERALITY_NOT_APPLICABLE*)
        ((= const 2) 'kortex_driver::ArmLaterality::*ARM_LATERALITY_LEFT*)
        ((= const 3) 'kortex_driver::ArmLaterality::*ARM_LATERALITY_RIGHT*)
        (t nil)))

(defclass kortex_driver::ArmLaterality
  :super ros::object
  :slots ())

(defmethod kortex_driver::ArmLaterality
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

(setf (get kortex_driver::ArmLaterality :md5sum-) "e87e70b626397c379fd01cdd30b3b822")
(setf (get kortex_driver::ArmLaterality :datatype-) "kortex_driver/ArmLaterality")
(setf (get kortex_driver::ArmLaterality :definition-)
      "
uint32 ARM_LATERALITY_UNSPECIFIED = 0

uint32 ARM_LATERALITY_NOT_APPLICABLE = 1

uint32 ARM_LATERALITY_LEFT = 2

uint32 ARM_LATERALITY_RIGHT = 3

")



(provide :kortex_driver/ArmLaterality "e87e70b626397c379fd01cdd30b3b822")


