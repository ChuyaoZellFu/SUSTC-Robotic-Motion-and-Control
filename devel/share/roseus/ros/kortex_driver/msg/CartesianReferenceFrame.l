;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianReferenceFrame)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianReferenceFrame (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME")
  (make-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))

(in-package "ROS")
;;//! \htmlinclude CartesianReferenceFrame.msg.html


(intern "*CARTESIAN_REFERENCE_FRAME_UNSPECIFIED*" (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(shadow '*CARTESIAN_REFERENCE_FRAME_UNSPECIFIED* (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(defconstant kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_UNSPECIFIED* 0)
(intern "*CARTESIAN_REFERENCE_FRAME_MIXED*" (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(shadow '*CARTESIAN_REFERENCE_FRAME_MIXED* (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(defconstant kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_MIXED* 1)
(intern "*CARTESIAN_REFERENCE_FRAME_TOOL*" (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(shadow '*CARTESIAN_REFERENCE_FRAME_TOOL* (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(defconstant kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_TOOL* 2)
(intern "*CARTESIAN_REFERENCE_FRAME_BASE*" (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(shadow '*CARTESIAN_REFERENCE_FRAME_BASE* (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAME"))
(defconstant kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_BASE* 3)

(defun kortex_driver::CartesianReferenceFrame-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_MIXED*)
        ((= const 2) 'kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_TOOL*)
        ((= const 3) 'kortex_driver::CartesianReferenceFrame::*CARTESIAN_REFERENCE_FRAME_BASE*)
        (t nil)))

(defclass kortex_driver::CartesianReferenceFrame
  :super ros::object
  :slots ())

(defmethod kortex_driver::CartesianReferenceFrame
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

(setf (get kortex_driver::CartesianReferenceFrame :md5sum-) "82b14912eebdea49eac0f3e20689bea8")
(setf (get kortex_driver::CartesianReferenceFrame :datatype-) "kortex_driver/CartesianReferenceFrame")
(setf (get kortex_driver::CartesianReferenceFrame :definition-)
      "
uint32 CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0

uint32 CARTESIAN_REFERENCE_FRAME_MIXED = 1

uint32 CARTESIAN_REFERENCE_FRAME_TOOL = 2

uint32 CARTESIAN_REFERENCE_FRAME_BASE = 3

")



(provide :kortex_driver/CartesianReferenceFrame "82b14912eebdea49eac0f3e20689bea8")


