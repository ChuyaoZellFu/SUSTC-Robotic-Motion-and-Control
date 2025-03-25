;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CalibrationParameter)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CalibrationParameter (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CALIBRATIONPARAMETER")
  (make-package "KORTEX_DRIVER::CALIBRATIONPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude CalibrationParameter.msg.html


(defclass kortex_driver::CalibrationParameter
  :super ros::object
  :slots (_calibration_parameter_identifier _oneof_value ))

(defmethod kortex_driver::CalibrationParameter
  (:init
   (&key
    ((:calibration_parameter_identifier __calibration_parameter_identifier) 0)
    ((:oneof_value __oneof_value) (instance kortex_driver::CalibrationParameter_value :init))
    )
   (send-super :init)
   (setq _calibration_parameter_identifier (round __calibration_parameter_identifier))
   (setq _oneof_value __oneof_value)
   self)
  (:calibration_parameter_identifier
   (&optional __calibration_parameter_identifier)
   (if __calibration_parameter_identifier (setq _calibration_parameter_identifier __calibration_parameter_identifier)) _calibration_parameter_identifier)
  (:oneof_value
   (&rest __oneof_value)
   (if (keywordp (car __oneof_value))
       (send* _oneof_value __oneof_value)
     (progn
       (if __oneof_value (setq _oneof_value (car __oneof_value)))
       _oneof_value)))
  (:serialization-length
   ()
   (+
    ;; uint32 _calibration_parameter_identifier
    4
    ;; kortex_driver/CalibrationParameter_value _oneof_value
    (send _oneof_value :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _calibration_parameter_identifier
       (write-long _calibration_parameter_identifier s)
     ;; kortex_driver/CalibrationParameter_value _oneof_value
       (send _oneof_value :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _calibration_parameter_identifier
     (setq _calibration_parameter_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/CalibrationParameter_value _oneof_value
     (send _oneof_value :deserialize buf ptr-) (incf ptr- (send _oneof_value :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::CalibrationParameter :md5sum-) "c9d31a3bff291563e3d32bb702a8bed2")
(setf (get kortex_driver::CalibrationParameter :datatype-) "kortex_driver/CalibrationParameter")
(setf (get kortex_driver::CalibrationParameter :definition-)
      "
uint32 calibration_parameter_identifier
CalibrationParameter_value oneof_value
================================================================================
MSG: kortex_driver/CalibrationParameter_value

uint32[] signedIntValue
uint32[] unsignedIntValue
uint32[] floatValue
")



(provide :kortex_driver/CalibrationParameter "c9d31a3bff291563e3d32bb702a8bed2")


