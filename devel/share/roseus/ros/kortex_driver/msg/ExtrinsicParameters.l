;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ExtrinsicParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ExtrinsicParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::EXTRINSICPARAMETERS")
  (make-package "KORTEX_DRIVER::EXTRINSICPARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude ExtrinsicParameters.msg.html


(defclass kortex_driver::ExtrinsicParameters
  :super ros::object
  :slots (_rotation _translation ))

(defmethod kortex_driver::ExtrinsicParameters
  (:init
   (&key
    ((:rotation __rotation) (instance kortex_driver::VisionConfig_RotationMatrix :init))
    ((:translation __translation) (instance kortex_driver::TranslationVector :init))
    )
   (send-super :init)
   (setq _rotation __rotation)
   (setq _translation __translation)
   self)
  (:rotation
   (&rest __rotation)
   (if (keywordp (car __rotation))
       (send* _rotation __rotation)
     (progn
       (if __rotation (setq _rotation (car __rotation)))
       _rotation)))
  (:translation
   (&rest __translation)
   (if (keywordp (car __translation))
       (send* _translation __translation)
     (progn
       (if __translation (setq _translation (car __translation)))
       _translation)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/VisionConfig_RotationMatrix _rotation
    (send _rotation :serialization-length)
    ;; kortex_driver/TranslationVector _translation
    (send _translation :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/VisionConfig_RotationMatrix _rotation
       (send _rotation :serialize s)
     ;; kortex_driver/TranslationVector _translation
       (send _translation :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/VisionConfig_RotationMatrix _rotation
     (send _rotation :deserialize buf ptr-) (incf ptr- (send _rotation :serialization-length))
   ;; kortex_driver/TranslationVector _translation
     (send _translation :deserialize buf ptr-) (incf ptr- (send _translation :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ExtrinsicParameters :md5sum-) "931ae3c2dcff1b69750ed6505e17f9d8")
(setf (get kortex_driver::ExtrinsicParameters :datatype-) "kortex_driver/ExtrinsicParameters")
(setf (get kortex_driver::ExtrinsicParameters :definition-)
      "
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
float32 t_z
")



(provide :kortex_driver/ExtrinsicParameters "931ae3c2dcff1b69750ed6505e17f9d8")


