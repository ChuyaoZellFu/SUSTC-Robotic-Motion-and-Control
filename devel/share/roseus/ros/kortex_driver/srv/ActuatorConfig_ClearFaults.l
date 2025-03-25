;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActuatorConfig_ClearFaults)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActuatorConfig_ClearFaults (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CLEARFAULTS")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_CLEARFAULTS"))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CLEARFAULTSREQUEST")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_CLEARFAULTSREQUEST"))
(unless (find-package "KORTEX_DRIVER::ACTUATORCONFIG_CLEARFAULTSRESPONSE")
  (make-package "KORTEX_DRIVER::ACTUATORCONFIG_CLEARFAULTSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ActuatorConfig_ClearFaultsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ActuatorConfig_ClearFaultsRequest
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

(defclass kortex_driver::ActuatorConfig_ClearFaultsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ActuatorConfig_ClearFaultsResponse
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

(defclass kortex_driver::ActuatorConfig_ClearFaults
  :super ros::object
  :slots ())

(setf (get kortex_driver::ActuatorConfig_ClearFaults :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ActuatorConfig_ClearFaults :datatype-) "kortex_driver/ActuatorConfig_ClearFaults")
(setf (get kortex_driver::ActuatorConfig_ClearFaults :request) kortex_driver::ActuatorConfig_ClearFaultsRequest)
(setf (get kortex_driver::ActuatorConfig_ClearFaults :response) kortex_driver::ActuatorConfig_ClearFaultsResponse)

(defmethod kortex_driver::ActuatorConfig_ClearFaultsRequest
  (:response () (instance kortex_driver::ActuatorConfig_ClearFaultsResponse :init)))

(setf (get kortex_driver::ActuatorConfig_ClearFaultsRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ActuatorConfig_ClearFaultsRequest :datatype-) "kortex_driver/ActuatorConfig_ClearFaultsRequest")
(setf (get kortex_driver::ActuatorConfig_ClearFaultsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ActuatorConfig_ClearFaultsResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ActuatorConfig_ClearFaultsResponse :datatype-) "kortex_driver/ActuatorConfig_ClearFaultsResponse")
(setf (get kortex_driver::ActuatorConfig_ClearFaultsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ActuatorConfig_ClearFaults "f335b819dc59099fe3124b36f140ad07")


