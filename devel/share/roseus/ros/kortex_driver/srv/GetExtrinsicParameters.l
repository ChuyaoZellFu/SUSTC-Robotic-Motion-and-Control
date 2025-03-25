;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetExtrinsicParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetExtrinsicParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETEXTRINSICPARAMETERS")
  (make-package "KORTEX_DRIVER::GETEXTRINSICPARAMETERS"))
(unless (find-package "KORTEX_DRIVER::GETEXTRINSICPARAMETERSREQUEST")
  (make-package "KORTEX_DRIVER::GETEXTRINSICPARAMETERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETEXTRINSICPARAMETERSRESPONSE")
  (make-package "KORTEX_DRIVER::GETEXTRINSICPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetExtrinsicParametersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetExtrinsicParametersRequest
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

(defclass kortex_driver::GetExtrinsicParametersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetExtrinsicParametersResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ExtrinsicParameters :init))
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
    ;; kortex_driver/ExtrinsicParameters _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ExtrinsicParameters _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ExtrinsicParameters _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetExtrinsicParameters
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetExtrinsicParameters :md5sum-) "769b6e0a4e2cbed0ce69ce84dc1b50e3")
(setf (get kortex_driver::GetExtrinsicParameters :datatype-) "kortex_driver/GetExtrinsicParameters")
(setf (get kortex_driver::GetExtrinsicParameters :request) kortex_driver::GetExtrinsicParametersRequest)
(setf (get kortex_driver::GetExtrinsicParameters :response) kortex_driver::GetExtrinsicParametersResponse)

(defmethod kortex_driver::GetExtrinsicParametersRequest
  (:response () (instance kortex_driver::GetExtrinsicParametersResponse :init)))

(setf (get kortex_driver::GetExtrinsicParametersRequest :md5sum-) "769b6e0a4e2cbed0ce69ce84dc1b50e3")
(setf (get kortex_driver::GetExtrinsicParametersRequest :datatype-) "kortex_driver/GetExtrinsicParametersRequest")
(setf (get kortex_driver::GetExtrinsicParametersRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ExtrinsicParameters output

================================================================================
MSG: kortex_driver/ExtrinsicParameters

VisionConfig_RotationMatrix rotation
TranslationVector translation
================================================================================
MSG: kortex_driver/VisionConfig_RotationMatrix

VisionConfig_RotationMatrixRow row1
VisionConfig_RotationMatrixRow row2
VisionConfig_RotationMatrixRow row3
================================================================================
MSG: kortex_driver/VisionConfig_RotationMatrixRow

float32 column1
float32 column2
float32 column3
================================================================================
MSG: kortex_driver/TranslationVector

float32 t_x
float32 t_y
")

(setf (get kortex_driver::GetExtrinsicParametersResponse :md5sum-) "769b6e0a4e2cbed0ce69ce84dc1b50e3")
(setf (get kortex_driver::GetExtrinsicParametersResponse :datatype-) "kortex_driver/GetExtrinsicParametersResponse")
(setf (get kortex_driver::GetExtrinsicParametersResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ExtrinsicParameters output

================================================================================
MSG: kortex_driver/ExtrinsicParameters

VisionConfig_RotationMatrix rotation
TranslationVector translation
================================================================================
MSG: kortex_driver/VisionConfig_RotationMatrix

VisionConfig_RotationMatrixRow row1
VisionConfig_RotationMatrixRow row2
VisionConfig_RotationMatrixRow row3
================================================================================
MSG: kortex_driver/VisionConfig_RotationMatrixRow

float32 column1
float32 column2
float32 column3
================================================================================
MSG: kortex_driver/TranslationVector

float32 t_x
float32 t_y
")



(provide :kortex_driver/GetExtrinsicParameters "769b6e0a4e2cbed0ce69ce84dc1b50e3")


