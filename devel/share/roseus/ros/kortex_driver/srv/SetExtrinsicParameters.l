;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetExtrinsicParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetExtrinsicParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETEXTRINSICPARAMETERS")
  (make-package "KORTEX_DRIVER::SETEXTRINSICPARAMETERS"))
(unless (find-package "KORTEX_DRIVER::SETEXTRINSICPARAMETERSREQUEST")
  (make-package "KORTEX_DRIVER::SETEXTRINSICPARAMETERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETEXTRINSICPARAMETERSRESPONSE")
  (make-package "KORTEX_DRIVER::SETEXTRINSICPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetExtrinsicParametersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetExtrinsicParametersRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ExtrinsicParameters :init))
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
    ;; kortex_driver/ExtrinsicParameters _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ExtrinsicParameters _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ExtrinsicParameters _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetExtrinsicParametersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetExtrinsicParametersResponse
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

(defclass kortex_driver::SetExtrinsicParameters
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetExtrinsicParameters :md5sum-) "4b2d31a314d526e9790309739cc1aeec")
(setf (get kortex_driver::SetExtrinsicParameters :datatype-) "kortex_driver/SetExtrinsicParameters")
(setf (get kortex_driver::SetExtrinsicParameters :request) kortex_driver::SetExtrinsicParametersRequest)
(setf (get kortex_driver::SetExtrinsicParameters :response) kortex_driver::SetExtrinsicParametersResponse)

(defmethod kortex_driver::SetExtrinsicParametersRequest
  (:response () (instance kortex_driver::SetExtrinsicParametersResponse :init)))

(setf (get kortex_driver::SetExtrinsicParametersRequest :md5sum-) "4b2d31a314d526e9790309739cc1aeec")
(setf (get kortex_driver::SetExtrinsicParametersRequest :datatype-) "kortex_driver/SetExtrinsicParametersRequest")
(setf (get kortex_driver::SetExtrinsicParametersRequest :definition-)
      "ExtrinsicParameters input

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
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetExtrinsicParametersResponse :md5sum-) "4b2d31a314d526e9790309739cc1aeec")
(setf (get kortex_driver::SetExtrinsicParametersResponse :datatype-) "kortex_driver/SetExtrinsicParametersResponse")
(setf (get kortex_driver::SetExtrinsicParametersResponse :definition-)
      "ExtrinsicParameters input

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
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetExtrinsicParameters "4b2d31a314d526e9790309739cc1aeec")


