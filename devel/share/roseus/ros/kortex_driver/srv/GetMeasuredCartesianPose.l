;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetMeasuredCartesianPose)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetMeasuredCartesianPose (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDCARTESIANPOSE")
  (make-package "KORTEX_DRIVER::GETMEASUREDCARTESIANPOSE"))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDCARTESIANPOSEREQUEST")
  (make-package "KORTEX_DRIVER::GETMEASUREDCARTESIANPOSEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDCARTESIANPOSERESPONSE")
  (make-package "KORTEX_DRIVER::GETMEASUREDCARTESIANPOSERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetMeasuredCartesianPoseRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetMeasuredCartesianPoseRequest
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

(defclass kortex_driver::GetMeasuredCartesianPoseResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetMeasuredCartesianPoseResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Pose :init))
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
    ;; kortex_driver/Pose _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Pose _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Pose _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetMeasuredCartesianPose
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetMeasuredCartesianPose :md5sum-) "4eaf9f25cbe269662009a9ad397d3f07")
(setf (get kortex_driver::GetMeasuredCartesianPose :datatype-) "kortex_driver/GetMeasuredCartesianPose")
(setf (get kortex_driver::GetMeasuredCartesianPose :request) kortex_driver::GetMeasuredCartesianPoseRequest)
(setf (get kortex_driver::GetMeasuredCartesianPose :response) kortex_driver::GetMeasuredCartesianPoseResponse)

(defmethod kortex_driver::GetMeasuredCartesianPoseRequest
  (:response () (instance kortex_driver::GetMeasuredCartesianPoseResponse :init)))

(setf (get kortex_driver::GetMeasuredCartesianPoseRequest :md5sum-) "4eaf9f25cbe269662009a9ad397d3f07")
(setf (get kortex_driver::GetMeasuredCartesianPoseRequest :datatype-) "kortex_driver/GetMeasuredCartesianPoseRequest")
(setf (get kortex_driver::GetMeasuredCartesianPoseRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Pose output

================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
")

(setf (get kortex_driver::GetMeasuredCartesianPoseResponse :md5sum-) "4eaf9f25cbe269662009a9ad397d3f07")
(setf (get kortex_driver::GetMeasuredCartesianPoseResponse :datatype-) "kortex_driver/GetMeasuredCartesianPoseResponse")
(setf (get kortex_driver::GetMeasuredCartesianPoseResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Pose output

================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
")



(provide :kortex_driver/GetMeasuredCartesianPose "4eaf9f25cbe269662009a9ad397d3f07")


