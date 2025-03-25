;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CalibrationResult)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CalibrationResult (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CALIBRATIONRESULT")
  (make-package "KORTEX_DRIVER::CALIBRATIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude CalibrationResult.msg.html


(defclass kortex_driver::CalibrationResult
  :super ros::object
  :slots (_calibration_status _calibration_details ))

(defmethod kortex_driver::CalibrationResult
  (:init
   (&key
    ((:calibration_status __calibration_status) 0)
    ((:calibration_details __calibration_details) 0)
    )
   (send-super :init)
   (setq _calibration_status (round __calibration_status))
   (setq _calibration_details (round __calibration_details))
   self)
  (:calibration_status
   (&optional __calibration_status)
   (if __calibration_status (setq _calibration_status __calibration_status)) _calibration_status)
  (:calibration_details
   (&optional __calibration_details)
   (if __calibration_details (setq _calibration_details __calibration_details)) _calibration_details)
  (:serialization-length
   ()
   (+
    ;; uint32 _calibration_status
    4
    ;; uint32 _calibration_details
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _calibration_status
       (write-long _calibration_status s)
     ;; uint32 _calibration_details
       (write-long _calibration_details s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _calibration_status
     (setq _calibration_status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _calibration_details
     (setq _calibration_details (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CalibrationResult :md5sum-) "3d6092aff91a3268dc7e1b71a52b5cd9")
(setf (get kortex_driver::CalibrationResult :datatype-) "kortex_driver/CalibrationResult")
(setf (get kortex_driver::CalibrationResult :definition-)
      "
uint32 calibration_status
uint32 calibration_details
")



(provide :kortex_driver/CalibrationResult "3d6092aff91a3268dc7e1b71a52b5cd9")


