;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetCalibrationResult)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetCalibrationResult (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCALIBRATIONRESULT")
  (make-package "KORTEX_DRIVER::GETCALIBRATIONRESULT"))
(unless (find-package "KORTEX_DRIVER::GETCALIBRATIONRESULTREQUEST")
  (make-package "KORTEX_DRIVER::GETCALIBRATIONRESULTREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCALIBRATIONRESULTRESPONSE")
  (make-package "KORTEX_DRIVER::GETCALIBRATIONRESULTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetCalibrationResultRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetCalibrationResultRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::CalibrationElement :init))
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
    ;; kortex_driver/CalibrationElement _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CalibrationElement _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CalibrationElement _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetCalibrationResultResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetCalibrationResultResponse
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

(defclass kortex_driver::GetCalibrationResult
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetCalibrationResult :md5sum-) "77b1f6ef7f1d5eace34aa81cf1772705")
(setf (get kortex_driver::GetCalibrationResult :datatype-) "kortex_driver/GetCalibrationResult")
(setf (get kortex_driver::GetCalibrationResult :request) kortex_driver::GetCalibrationResultRequest)
(setf (get kortex_driver::GetCalibrationResult :response) kortex_driver::GetCalibrationResultResponse)

(defmethod kortex_driver::GetCalibrationResultRequest
  (:response () (instance kortex_driver::GetCalibrationResultResponse :init)))

(setf (get kortex_driver::GetCalibrationResultRequest :md5sum-) "77b1f6ef7f1d5eace34aa81cf1772705")
(setf (get kortex_driver::GetCalibrationResultRequest :datatype-) "kortex_driver/GetCalibrationResultRequest")
(setf (get kortex_driver::GetCalibrationResultRequest :definition-)
      "CalibrationElement input

================================================================================
MSG: kortex_driver/CalibrationElement

---
CalibrationResult output

================================================================================
MSG: kortex_driver/CalibrationResult

uint32 calibration_status
")

(setf (get kortex_driver::GetCalibrationResultResponse :md5sum-) "77b1f6ef7f1d5eace34aa81cf1772705")
(setf (get kortex_driver::GetCalibrationResultResponse :datatype-) "kortex_driver/GetCalibrationResultResponse")
(setf (get kortex_driver::GetCalibrationResultResponse :definition-)
      "CalibrationElement input

================================================================================
MSG: kortex_driver/CalibrationElement

---
CalibrationResult output

================================================================================
MSG: kortex_driver/CalibrationResult

uint32 calibration_status
")



(provide :kortex_driver/GetCalibrationResult "77b1f6ef7f1d5eace34aa81cf1772705")


