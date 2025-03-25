;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ExecuteCalibration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ExecuteCalibration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::EXECUTECALIBRATION")
  (make-package "KORTEX_DRIVER::EXECUTECALIBRATION"))
(unless (find-package "KORTEX_DRIVER::EXECUTECALIBRATIONREQUEST")
  (make-package "KORTEX_DRIVER::EXECUTECALIBRATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::EXECUTECALIBRATIONRESPONSE")
  (make-package "KORTEX_DRIVER::EXECUTECALIBRATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ExecuteCalibrationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ExecuteCalibrationRequest
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

(defclass kortex_driver::ExecuteCalibrationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ExecuteCalibrationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ExecuteCalibration
  :super ros::object
  :slots ())

(setf (get kortex_driver::ExecuteCalibration :md5sum-) "8aabe5c823e6d1cd3d6eddd247265dae")
(setf (get kortex_driver::ExecuteCalibration :datatype-) "kortex_driver/ExecuteCalibration")
(setf (get kortex_driver::ExecuteCalibration :request) kortex_driver::ExecuteCalibrationRequest)
(setf (get kortex_driver::ExecuteCalibration :response) kortex_driver::ExecuteCalibrationResponse)

(defmethod kortex_driver::ExecuteCalibrationRequest
  (:response () (instance kortex_driver::ExecuteCalibrationResponse :init)))

(setf (get kortex_driver::ExecuteCalibrationRequest :md5sum-) "8aabe5c823e6d1cd3d6eddd247265dae")
(setf (get kortex_driver::ExecuteCalibrationRequest :datatype-) "kortex_driver/ExecuteCalibrationRequest")
(setf (get kortex_driver::ExecuteCalibrationRequest :definition-)
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
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ExecuteCalibrationResponse :md5sum-) "8aabe5c823e6d1cd3d6eddd247265dae")
(setf (get kortex_driver::ExecuteCalibrationResponse :datatype-) "kortex_driver/ExecuteCalibrationResponse")
(setf (get kortex_driver::ExecuteCalibrationResponse :definition-)
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
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ExecuteCalibration "8aabe5c823e6d1cd3d6eddd247265dae")


