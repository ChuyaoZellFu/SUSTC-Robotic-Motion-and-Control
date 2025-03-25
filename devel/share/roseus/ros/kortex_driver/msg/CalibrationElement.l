;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CalibrationElement)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CalibrationElement (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CALIBRATIONELEMENT")
  (make-package "KORTEX_DRIVER::CALIBRATIONELEMENT"))

(in-package "ROS")
;;//! \htmlinclude CalibrationElement.msg.html


(defclass kortex_driver::CalibrationElement
  :super ros::object
  :slots (_calibration_item ))

(defmethod kortex_driver::CalibrationElement
  (:init
   (&key
    ((:calibration_item __calibration_item) 0)
    )
   (send-super :init)
   (setq _calibration_item (round __calibration_item))
   self)
  (:calibration_item
   (&optional __calibration_item)
   (if __calibration_item (setq _calibration_item __calibration_item)) _calibration_item)
  (:serialization-length
   ()
   (+
    ;; uint32 _calibration_item
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _calibration_item
       (write-long _calibration_item s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _calibration_item
     (setq _calibration_item (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CalibrationElement :md5sum-) "48b2b6bb8d2654cfbe2379c075bcb6a4")
(setf (get kortex_driver::CalibrationElement :datatype-) "kortex_driver/CalibrationElement")
(setf (get kortex_driver::CalibrationElement :definition-)
      "
uint32 calibration_item
")



(provide :kortex_driver/CalibrationElement "48b2b6bb8d2654cfbe2379c075bcb6a4")


