;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Calibration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Calibration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CALIBRATION")
  (make-package "KORTEX_DRIVER::CALIBRATION"))

(in-package "ROS")
;;//! \htmlinclude Calibration.msg.html


(defclass kortex_driver::Calibration
  :super ros::object
  :slots (_calibration_item _calibration_parameter ))

(defmethod kortex_driver::Calibration
  (:init
   (&key
    ((:calibration_item __calibration_item) 0)
    ((:calibration_parameter __calibration_parameter) ())
    )
   (send-super :init)
   (setq _calibration_item (round __calibration_item))
   (setq _calibration_parameter __calibration_parameter)
   self)
  (:calibration_item
   (&optional __calibration_item)
   (if __calibration_item (setq _calibration_item __calibration_item)) _calibration_item)
  (:calibration_parameter
   (&rest __calibration_parameter)
   (if (keywordp (car __calibration_parameter))
       (send* _calibration_parameter __calibration_parameter)
     (progn
       (if __calibration_parameter (setq _calibration_parameter (car __calibration_parameter)))
       _calibration_parameter)))
  (:serialization-length
   ()
   (+
    ;; uint32 _calibration_item
    4
    ;; kortex_driver/CalibrationParameter[] _calibration_parameter
    (apply #'+ (send-all _calibration_parameter :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _calibration_item
       (write-long _calibration_item s)
     ;; kortex_driver/CalibrationParameter[] _calibration_parameter
     (write-long (length _calibration_parameter) s)
     (dolist (elem _calibration_parameter)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _calibration_item
     (setq _calibration_item (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/CalibrationParameter[] _calibration_parameter
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _calibration_parameter (let (r) (dotimes (i n) (push (instance kortex_driver::CalibrationParameter :init) r)) r))
     (dolist (elem- _calibration_parameter)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::Calibration :md5sum-) "4e1f62c964640a6b04e3e0de4bfef9f2")
(setf (get kortex_driver::Calibration :datatype-) "kortex_driver/Calibration")
(setf (get kortex_driver::Calibration :definition-)
      "
uint32 calibration_item
CalibrationParameter[] calibration_parameter
================================================================================
MSG: kortex_driver/CalibrationParameter

uint32 calibration_parameter_identifier
CalibrationParameter_value oneof_value
================================================================================
MSG: kortex_driver/CalibrationParameter_value

uint32[] signedIntValue
uint32[] unsignedIntValue
uint32[] floatValue
")



(provide :kortex_driver/Calibration "4e1f62c964640a6b04e3e0de4bfef9f2")


