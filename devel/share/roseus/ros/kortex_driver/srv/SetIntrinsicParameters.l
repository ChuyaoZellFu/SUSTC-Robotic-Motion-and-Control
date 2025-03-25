;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetIntrinsicParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetIntrinsicParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETINTRINSICPARAMETERS")
  (make-package "KORTEX_DRIVER::SETINTRINSICPARAMETERS"))
(unless (find-package "KORTEX_DRIVER::SETINTRINSICPARAMETERSREQUEST")
  (make-package "KORTEX_DRIVER::SETINTRINSICPARAMETERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETINTRINSICPARAMETERSRESPONSE")
  (make-package "KORTEX_DRIVER::SETINTRINSICPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetIntrinsicParametersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetIntrinsicParametersRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::IntrinsicParameters :init))
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
    ;; kortex_driver/IntrinsicParameters _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/IntrinsicParameters _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/IntrinsicParameters _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetIntrinsicParametersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetIntrinsicParametersResponse
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

(defclass kortex_driver::SetIntrinsicParameters
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetIntrinsicParameters :md5sum-) "e4fa2fa37ca4b5105af9a0b157505e8c")
(setf (get kortex_driver::SetIntrinsicParameters :datatype-) "kortex_driver/SetIntrinsicParameters")
(setf (get kortex_driver::SetIntrinsicParameters :request) kortex_driver::SetIntrinsicParametersRequest)
(setf (get kortex_driver::SetIntrinsicParameters :response) kortex_driver::SetIntrinsicParametersResponse)

(defmethod kortex_driver::SetIntrinsicParametersRequest
  (:response () (instance kortex_driver::SetIntrinsicParametersResponse :init)))

(setf (get kortex_driver::SetIntrinsicParametersRequest :md5sum-) "e4fa2fa37ca4b5105af9a0b157505e8c")
(setf (get kortex_driver::SetIntrinsicParametersRequest :datatype-) "kortex_driver/SetIntrinsicParametersRequest")
(setf (get kortex_driver::SetIntrinsicParametersRequest :definition-)
      "IntrinsicParameters input

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
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetIntrinsicParametersResponse :md5sum-) "e4fa2fa37ca4b5105af9a0b157505e8c")
(setf (get kortex_driver::SetIntrinsicParametersResponse :datatype-) "kortex_driver/SetIntrinsicParametersResponse")
(setf (get kortex_driver::SetIntrinsicParametersResponse :definition-)
      "IntrinsicParameters input

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
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetIntrinsicParameters "e4fa2fa37ca4b5105af9a0b157505e8c")


