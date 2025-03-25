;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetIntrinsicParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetIntrinsicParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETINTRINSICPARAMETERS")
  (make-package "KORTEX_DRIVER::GETINTRINSICPARAMETERS"))
(unless (find-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSREQUEST")
  (make-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSRESPONSE")
  (make-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetIntrinsicParametersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetIntrinsicParametersRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SensorIdentifier :init))
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
    ;; kortex_driver/SensorIdentifier _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SensorIdentifier _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SensorIdentifier _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetIntrinsicParametersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetIntrinsicParametersResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::IntrinsicParameters :init))
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
    ;; kortex_driver/IntrinsicParameters _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/IntrinsicParameters _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/IntrinsicParameters _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetIntrinsicParameters
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetIntrinsicParameters :md5sum-) "b088fe2711ccce33e17ff2a6a6244b06")
(setf (get kortex_driver::GetIntrinsicParameters :datatype-) "kortex_driver/GetIntrinsicParameters")
(setf (get kortex_driver::GetIntrinsicParameters :request) kortex_driver::GetIntrinsicParametersRequest)
(setf (get kortex_driver::GetIntrinsicParameters :response) kortex_driver::GetIntrinsicParametersResponse)

(defmethod kortex_driver::GetIntrinsicParametersRequest
  (:response () (instance kortex_driver::GetIntrinsicParametersResponse :init)))

(setf (get kortex_driver::GetIntrinsicParametersRequest :md5sum-) "b088fe2711ccce33e17ff2a6a6244b06")
(setf (get kortex_driver::GetIntrinsicParametersRequest :datatype-) "kortex_driver/GetIntrinsicParametersRequest")
(setf (get kortex_driver::GetIntrinsicParametersRequest :definition-)
      "SensorIdentifier input

================================================================================
MSG: kortex_driver/SensorIdentifier

---
IntrinsicParameters output

================================================================================
MSG: kortex_driver/IntrinsicParameters

uint32 sensor
uint32 resolution
float32 principal_point_x
float32 principal_point_y
float32 focal_length_x
float32 focal_length_y
DistortionCoefficients distortion_coeffs
================================================================================
MSG: kortex_driver/DistortionCoefficients

float32 k1
float32 k2
float32 k3
float32 p1
")

(setf (get kortex_driver::GetIntrinsicParametersResponse :md5sum-) "b088fe2711ccce33e17ff2a6a6244b06")
(setf (get kortex_driver::GetIntrinsicParametersResponse :datatype-) "kortex_driver/GetIntrinsicParametersResponse")
(setf (get kortex_driver::GetIntrinsicParametersResponse :definition-)
      "SensorIdentifier input

================================================================================
MSG: kortex_driver/SensorIdentifier

---
IntrinsicParameters output

================================================================================
MSG: kortex_driver/IntrinsicParameters

uint32 sensor
uint32 resolution
float32 principal_point_x
float32 principal_point_y
float32 focal_length_x
float32 focal_length_y
DistortionCoefficients distortion_coeffs
================================================================================
MSG: kortex_driver/DistortionCoefficients

float32 k1
float32 k2
float32 k3
float32 p1
")



(provide :kortex_driver/GetIntrinsicParameters "b088fe2711ccce33e17ff2a6a6244b06")


