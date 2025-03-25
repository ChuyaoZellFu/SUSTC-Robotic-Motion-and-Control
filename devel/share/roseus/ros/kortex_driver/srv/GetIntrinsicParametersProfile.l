;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetIntrinsicParametersProfile)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetIntrinsicParametersProfile (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSPROFILE")
  (make-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSPROFILE"))
(unless (find-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSPROFILEREQUEST")
  (make-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSPROFILEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSPROFILERESPONSE")
  (make-package "KORTEX_DRIVER::GETINTRINSICPARAMETERSPROFILERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetIntrinsicParametersProfileRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetIntrinsicParametersProfileRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::IntrinsicProfileIdentifier :init))
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
    ;; kortex_driver/IntrinsicProfileIdentifier _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/IntrinsicProfileIdentifier _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/IntrinsicProfileIdentifier _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetIntrinsicParametersProfileResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetIntrinsicParametersProfileResponse
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

(defclass kortex_driver::GetIntrinsicParametersProfile
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetIntrinsicParametersProfile :md5sum-) "9b534af230711bb30b60153a5b06f949")
(setf (get kortex_driver::GetIntrinsicParametersProfile :datatype-) "kortex_driver/GetIntrinsicParametersProfile")
(setf (get kortex_driver::GetIntrinsicParametersProfile :request) kortex_driver::GetIntrinsicParametersProfileRequest)
(setf (get kortex_driver::GetIntrinsicParametersProfile :response) kortex_driver::GetIntrinsicParametersProfileResponse)

(defmethod kortex_driver::GetIntrinsicParametersProfileRequest
  (:response () (instance kortex_driver::GetIntrinsicParametersProfileResponse :init)))

(setf (get kortex_driver::GetIntrinsicParametersProfileRequest :md5sum-) "9b534af230711bb30b60153a5b06f949")
(setf (get kortex_driver::GetIntrinsicParametersProfileRequest :datatype-) "kortex_driver/GetIntrinsicParametersProfileRequest")
(setf (get kortex_driver::GetIntrinsicParametersProfileRequest :definition-)
      "IntrinsicProfileIdentifier input

================================================================================
MSG: kortex_driver/IntrinsicProfileIdentifier

uint32 sensor
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

(setf (get kortex_driver::GetIntrinsicParametersProfileResponse :md5sum-) "9b534af230711bb30b60153a5b06f949")
(setf (get kortex_driver::GetIntrinsicParametersProfileResponse :datatype-) "kortex_driver/GetIntrinsicParametersProfileResponse")
(setf (get kortex_driver::GetIntrinsicParametersProfileResponse :definition-)
      "IntrinsicProfileIdentifier input

================================================================================
MSG: kortex_driver/IntrinsicProfileIdentifier

uint32 sensor
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



(provide :kortex_driver/GetIntrinsicParametersProfile "9b534af230711bb30b60153a5b06f949")


