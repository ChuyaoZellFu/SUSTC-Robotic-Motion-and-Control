;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RestoreFactoryProductConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RestoreFactoryProductConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESTOREFACTORYPRODUCTCONFIGURATION")
  (make-package "KORTEX_DRIVER::RESTOREFACTORYPRODUCTCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::RESTOREFACTORYPRODUCTCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::RESTOREFACTORYPRODUCTCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESTOREFACTORYPRODUCTCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::RESTOREFACTORYPRODUCTCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::RestoreFactoryProductConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::RestoreFactoryProductConfigurationRequest
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

(defclass kortex_driver::RestoreFactoryProductConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::RestoreFactoryProductConfigurationResponse
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

(defclass kortex_driver::RestoreFactoryProductConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::RestoreFactoryProductConfiguration :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::RestoreFactoryProductConfiguration :datatype-) "kortex_driver/RestoreFactoryProductConfiguration")
(setf (get kortex_driver::RestoreFactoryProductConfiguration :request) kortex_driver::RestoreFactoryProductConfigurationRequest)
(setf (get kortex_driver::RestoreFactoryProductConfiguration :response) kortex_driver::RestoreFactoryProductConfigurationResponse)

(defmethod kortex_driver::RestoreFactoryProductConfigurationRequest
  (:response () (instance kortex_driver::RestoreFactoryProductConfigurationResponse :init)))

(setf (get kortex_driver::RestoreFactoryProductConfigurationRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::RestoreFactoryProductConfigurationRequest :datatype-) "kortex_driver/RestoreFactoryProductConfigurationRequest")
(setf (get kortex_driver::RestoreFactoryProductConfigurationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::RestoreFactoryProductConfigurationResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::RestoreFactoryProductConfigurationResponse :datatype-) "kortex_driver/RestoreFactoryProductConfigurationResponse")
(setf (get kortex_driver::RestoreFactoryProductConfigurationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/RestoreFactoryProductConfiguration "f335b819dc59099fe3124b36f140ad07")


