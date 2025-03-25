;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CalibrationStatus)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CalibrationStatus (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS")
  (make-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude CalibrationStatus.msg.html


(intern "*UNSPECIFIED_CALIBRATION_STATUS*" (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(shadow '*UNSPECIFIED_CALIBRATION_STATUS* (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(defconstant kortex_driver::CalibrationStatus::*UNSPECIFIED_CALIBRATION_STATUS* 0)
(intern "*NOT_CALIBRATED*" (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(shadow '*NOT_CALIBRATED* (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(defconstant kortex_driver::CalibrationStatus::*NOT_CALIBRATED* 1)
(intern "*IN_PROGRESS*" (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(shadow '*IN_PROGRESS* (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(defconstant kortex_driver::CalibrationStatus::*IN_PROGRESS* 2)
(intern "*CALIBRATED*" (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(shadow '*CALIBRATED* (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(defconstant kortex_driver::CalibrationStatus::*CALIBRATED* 3)
(intern "*IN_FAULT*" (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(shadow '*IN_FAULT* (find-package "KORTEX_DRIVER::CALIBRATIONSTATUS"))
(defconstant kortex_driver::CalibrationStatus::*IN_FAULT* 4)

(defun kortex_driver::CalibrationStatus-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::CalibrationStatus::*UNSPECIFIED_CALIBRATION_STATUS*)
        ((= const 1) 'kortex_driver::CalibrationStatus::*NOT_CALIBRATED*)
        ((= const 2) 'kortex_driver::CalibrationStatus::*IN_PROGRESS*)
        ((= const 3) 'kortex_driver::CalibrationStatus::*CALIBRATED*)
        ((= const 4) 'kortex_driver::CalibrationStatus::*IN_FAULT*)
        (t nil)))

(defclass kortex_driver::CalibrationStatus
  :super ros::object
  :slots ())

(defmethod kortex_driver::CalibrationStatus
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

(setf (get kortex_driver::CalibrationStatus :md5sum-) "a4d9ce00135ee251dc1d3b34f761f5f8")
(setf (get kortex_driver::CalibrationStatus :datatype-) "kortex_driver/CalibrationStatus")
(setf (get kortex_driver::CalibrationStatus :definition-)
      "
uint32 UNSPECIFIED_CALIBRATION_STATUS = 0

uint32 NOT_CALIBRATED = 1

uint32 IN_PROGRESS = 2

uint32 CALIBRATED = 3

uint32 IN_FAULT = 4

")



(provide :kortex_driver/CalibrationStatus "a4d9ce00135ee251dc1d3b34f761f5f8")


