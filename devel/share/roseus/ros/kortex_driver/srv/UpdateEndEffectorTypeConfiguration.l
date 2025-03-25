;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UpdateEndEffectorTypeConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UpdateEndEffectorTypeConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UPDATEENDEFFECTORTYPECONFIGURATION")
  (make-package "KORTEX_DRIVER::UPDATEENDEFFECTORTYPECONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::UPDATEENDEFFECTORTYPECONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::UPDATEENDEFFECTORTYPECONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::UPDATEENDEFFECTORTYPECONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::UPDATEENDEFFECTORTYPECONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::UpdateEndEffectorTypeConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::UpdateEndEffectorTypeConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ProductConfigurationEndEffectorType :init))
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
    ;; kortex_driver/ProductConfigurationEndEffectorType _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProductConfigurationEndEffectorType _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProductConfigurationEndEffectorType _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::UpdateEndEffectorTypeConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::UpdateEndEffectorTypeConfigurationResponse
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

(defclass kortex_driver::UpdateEndEffectorTypeConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::UpdateEndEffectorTypeConfiguration :md5sum-) "f022a6fb3cba8e0906c5a35f2f76f468")
(setf (get kortex_driver::UpdateEndEffectorTypeConfiguration :datatype-) "kortex_driver/UpdateEndEffectorTypeConfiguration")
(setf (get kortex_driver::UpdateEndEffectorTypeConfiguration :request) kortex_driver::UpdateEndEffectorTypeConfigurationRequest)
(setf (get kortex_driver::UpdateEndEffectorTypeConfiguration :response) kortex_driver::UpdateEndEffectorTypeConfigurationResponse)

(defmethod kortex_driver::UpdateEndEffectorTypeConfigurationRequest
  (:response () (instance kortex_driver::UpdateEndEffectorTypeConfigurationResponse :init)))

(setf (get kortex_driver::UpdateEndEffectorTypeConfigurationRequest :md5sum-) "f022a6fb3cba8e0906c5a35f2f76f468")
(setf (get kortex_driver::UpdateEndEffectorTypeConfigurationRequest :datatype-) "kortex_driver/UpdateEndEffectorTypeConfigurationRequest")
(setf (get kortex_driver::UpdateEndEffectorTypeConfigurationRequest :definition-)
      "ProductConfigurationEndEffectorType input

================================================================================
MSG: kortex_driver/ProductConfigurationEndEffectorType

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::UpdateEndEffectorTypeConfigurationResponse :md5sum-) "f022a6fb3cba8e0906c5a35f2f76f468")
(setf (get kortex_driver::UpdateEndEffectorTypeConfigurationResponse :datatype-) "kortex_driver/UpdateEndEffectorTypeConfigurationResponse")
(setf (get kortex_driver::UpdateEndEffectorTypeConfigurationResponse :definition-)
      "ProductConfigurationEndEffectorType input

================================================================================
MSG: kortex_driver/ProductConfigurationEndEffectorType

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/UpdateEndEffectorTypeConfiguration "f022a6fb3cba8e0906c5a35f2f76f468")


