;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StopCalibration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StopCalibration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STOPCALIBRATION")
  (make-package "KORTEX_DRIVER::STOPCALIBRATION"))
(unless (find-package "KORTEX_DRIVER::STOPCALIBRATIONREQUEST")
  (make-package "KORTEX_DRIVER::STOPCALIBRATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::STOPCALIBRATIONRESPONSE")
  (make-package "KORTEX_DRIVER::STOPCALIBRATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::StopCalibrationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::StopCalibrationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Calibration :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Calibration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Calibration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Calibration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::StopCalibrationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::StopCalibrationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CalibrationResult :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/CalibrationResult _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CalibrationResult _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CalibrationResult _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::StopCalibration
  :super ros::object
  :slots ())

(setf (get kortex_driver::StopCalibration :md5sum-) "d0dfa8caa770ffbbffd12276bb3d7f2c")
(setf (get kortex_driver::StopCalibration :datatype-) "kortex_driver/StopCalibration")
(setf (get kortex_driver::StopCalibration :request) kortex_driver::StopCalibrationRequest)
(setf (get kortex_driver::StopCalibration :response) kortex_driver::StopCalibrationResponse)

(defmethod kortex_driver::StopCalibrationRequest
  (:response () (instance kortex_driver::StopCalibrationResponse :init)))

(setf (get kortex_driver::StopCalibrationRequest :md5sum-) "d0dfa8caa770ffbbffd12276bb3d7f2c")
(setf (get kortex_driver::StopCalibrationRequest :datatype-) "kortex_driver/StopCalibrationRequest")
(setf (get kortex_driver::StopCalibrationRequest :definition-)
      "Calibration input

================================================================================
MSG: kortex_driver/Calibration

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
---
CalibrationResult output

================================================================================
MSG: kortex_driver/CalibrationResult

uint32 calibration_status
")

(setf (get kortex_driver::StopCalibrationResponse :md5sum-) "d0dfa8caa770ffbbffd12276bb3d7f2c")
(setf (get kortex_driver::StopCalibrationResponse :datatype-) "kortex_driver/StopCalibrationResponse")
(setf (get kortex_driver::StopCalibrationResponse :definition-)
      "Calibration input

================================================================================
MSG: kortex_driver/Calibration

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
---
CalibrationResult output

================================================================================
MSG: kortex_driver/CalibrationResult

uint32 calibration_status
")



(provide :kortex_driver/StopCalibration "d0dfa8caa770ffbbffd12276bb3d7f2c")


