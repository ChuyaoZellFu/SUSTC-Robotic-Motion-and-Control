;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CalibrationItem)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CalibrationItem (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CALIBRATIONITEM")
  (make-package "KORTEX_DRIVER::CALIBRATIONITEM"))

(in-package "ROS")
;;//! \htmlinclude CalibrationItem.msg.html


(intern "*UNSPECIFIED_CALIBRATION_ITEM*" (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(shadow '*UNSPECIFIED_CALIBRATION_ITEM* (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(defconstant kortex_driver::CalibrationItem::*UNSPECIFIED_CALIBRATION_ITEM* 0)
(intern "*COGGING*" (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(shadow '*COGGING* (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(defconstant kortex_driver::CalibrationItem::*COGGING* 1)
(intern "*MAGNETIC*" (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(shadow '*MAGNETIC* (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(defconstant kortex_driver::CalibrationItem::*MAGNETIC* 2)
(intern "*MOTOR*" (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(shadow '*MOTOR* (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(defconstant kortex_driver::CalibrationItem::*MOTOR* 3)
(intern "*POSITION_RANGE*" (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(shadow '*POSITION_RANGE* (find-package "KORTEX_DRIVER::CALIBRATIONITEM"))
(defconstant kortex_driver::CalibrationItem::*POSITION_RANGE* 4)

(defun kortex_driver::CalibrationItem-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::CalibrationItem::*UNSPECIFIED_CALIBRATION_ITEM*)
        ((= const 1) 'kortex_driver::CalibrationItem::*COGGING*)
        ((= const 2) 'kortex_driver::CalibrationItem::*MAGNETIC*)
        ((= const 3) 'kortex_driver::CalibrationItem::*MOTOR*)
        ((= const 4) 'kortex_driver::CalibrationItem::*POSITION_RANGE*)
        (t nil)))

(defclass kortex_driver::CalibrationItem
  :super ros::object
  :slots ())

(defmethod kortex_driver::CalibrationItem
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

(setf (get kortex_driver::CalibrationItem :md5sum-) "004ebe034d74d471c7b919bf63c0f51a")
(setf (get kortex_driver::CalibrationItem :datatype-) "kortex_driver/CalibrationItem")
(setf (get kortex_driver::CalibrationItem :definition-)
      "
uint32 UNSPECIFIED_CALIBRATION_ITEM = 0

uint32 COGGING = 1

uint32 MAGNETIC = 2

uint32 MOTOR = 3

uint32 POSITION_RANGE = 4

")



(provide :kortex_driver/CalibrationItem "004ebe034d74d471c7b919bf63c0f51a")


