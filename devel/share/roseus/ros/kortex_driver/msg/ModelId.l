;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ModelId)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ModelId (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MODELID")
  (make-package "KORTEX_DRIVER::MODELID"))

(in-package "ROS")
;;//! \htmlinclude ModelId.msg.html


(intern "*MODEL_ID_UNSPECIFIED*" (find-package "KORTEX_DRIVER::MODELID"))
(shadow '*MODEL_ID_UNSPECIFIED* (find-package "KORTEX_DRIVER::MODELID"))
(defconstant kortex_driver::ModelId::*MODEL_ID_UNSPECIFIED* 0)
(intern "*MODEL_ID_L53*" (find-package "KORTEX_DRIVER::MODELID"))
(shadow '*MODEL_ID_L53* (find-package "KORTEX_DRIVER::MODELID"))
(defconstant kortex_driver::ModelId::*MODEL_ID_L53* 1)
(intern "*MODEL_ID_L31*" (find-package "KORTEX_DRIVER::MODELID"))
(shadow '*MODEL_ID_L31* (find-package "KORTEX_DRIVER::MODELID"))
(defconstant kortex_driver::ModelId::*MODEL_ID_L31* 2)
(intern "*MODEL_ID_HDK*" (find-package "KORTEX_DRIVER::MODELID"))
(shadow '*MODEL_ID_HDK* (find-package "KORTEX_DRIVER::MODELID"))
(defconstant kortex_driver::ModelId::*MODEL_ID_HDK* 3)

(defun kortex_driver::ModelId-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::ModelId::*MODEL_ID_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::ModelId::*MODEL_ID_L53*)
        ((= const 2) 'kortex_driver::ModelId::*MODEL_ID_L31*)
        ((= const 3) 'kortex_driver::ModelId::*MODEL_ID_HDK*)
        (t nil)))

(defclass kortex_driver::ModelId
  :super ros::object
  :slots ())

(defmethod kortex_driver::ModelId
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

(setf (get kortex_driver::ModelId :md5sum-) "7fb7d718b4a478fa436cc49becd077b0")
(setf (get kortex_driver::ModelId :datatype-) "kortex_driver/ModelId")
(setf (get kortex_driver::ModelId :definition-)
      "
uint32 MODEL_ID_UNSPECIFIED = 0

uint32 MODEL_ID_L53 = 1

uint32 MODEL_ID_L31 = 2

uint32 MODEL_ID_HDK = 3

")



(provide :kortex_driver/ModelId "7fb7d718b4a478fa436cc49becd077b0")


