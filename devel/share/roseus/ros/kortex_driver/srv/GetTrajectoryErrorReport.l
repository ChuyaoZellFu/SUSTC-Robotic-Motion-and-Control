;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetTrajectoryErrorReport)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetTrajectoryErrorReport (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETTRAJECTORYERRORREPORT")
  (make-package "KORTEX_DRIVER::GETTRAJECTORYERRORREPORT"))
(unless (find-package "KORTEX_DRIVER::GETTRAJECTORYERRORREPORTREQUEST")
  (make-package "KORTEX_DRIVER::GETTRAJECTORYERRORREPORTREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETTRAJECTORYERRORREPORTRESPONSE")
  (make-package "KORTEX_DRIVER::GETTRAJECTORYERRORREPORTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetTrajectoryErrorReportRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetTrajectoryErrorReportRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetTrajectoryErrorReportResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetTrajectoryErrorReportResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::TrajectoryErrorReport :init))
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
    ;; kortex_driver/TrajectoryErrorReport _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TrajectoryErrorReport _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TrajectoryErrorReport _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetTrajectoryErrorReport
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetTrajectoryErrorReport :md5sum-) "256dc258079fa086775c57ef6b093f4a")
(setf (get kortex_driver::GetTrajectoryErrorReport :datatype-) "kortex_driver/GetTrajectoryErrorReport")
(setf (get kortex_driver::GetTrajectoryErrorReport :request) kortex_driver::GetTrajectoryErrorReportRequest)
(setf (get kortex_driver::GetTrajectoryErrorReport :response) kortex_driver::GetTrajectoryErrorReportResponse)

(defmethod kortex_driver::GetTrajectoryErrorReportRequest
  (:response () (instance kortex_driver::GetTrajectoryErrorReportResponse :init)))

(setf (get kortex_driver::GetTrajectoryErrorReportRequest :md5sum-) "256dc258079fa086775c57ef6b093f4a")
(setf (get kortex_driver::GetTrajectoryErrorReportRequest :datatype-) "kortex_driver/GetTrajectoryErrorReportRequest")
(setf (get kortex_driver::GetTrajectoryErrorReportRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
TrajectoryErrorReport output

================================================================================
MSG: kortex_driver/TrajectoryErrorReport

TrajectoryErrorElement[] trajectory_error_elements
================================================================================
MSG: kortex_driver/TrajectoryErrorElement

uint32 error_type
uint32 error_identifier
float32 error_value
float32 min_value
float32 max_value
uint32 index
string message
")

(setf (get kortex_driver::GetTrajectoryErrorReportResponse :md5sum-) "256dc258079fa086775c57ef6b093f4a")
(setf (get kortex_driver::GetTrajectoryErrorReportResponse :datatype-) "kortex_driver/GetTrajectoryErrorReportResponse")
(setf (get kortex_driver::GetTrajectoryErrorReportResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
TrajectoryErrorReport output

================================================================================
MSG: kortex_driver/TrajectoryErrorReport

TrajectoryErrorElement[] trajectory_error_elements
================================================================================
MSG: kortex_driver/TrajectoryErrorElement

uint32 error_type
uint32 error_identifier
float32 error_value
float32 min_value
float32 max_value
uint32 index
string message
")



(provide :kortex_driver/GetTrajectoryErrorReport "256dc258079fa086775c57ef6b093f4a")


