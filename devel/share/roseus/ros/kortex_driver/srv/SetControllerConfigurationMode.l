;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetControllerConfigurationMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetControllerConfigurationMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONMODE")
  (make-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONMODE"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONMODEREQUEST")
  (make-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONMODERESPONSE")
  (make-package "KORTEX_DRIVER::SETCONTROLLERCONFIGURATIONMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetControllerConfigurationModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetControllerConfigurationModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControllerConfigurationMode :init))
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
    ;; kortex_driver/ControllerConfigurationMode _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerConfigurationMode _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerConfigurationMode _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetControllerConfigurationModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetControllerConfigurationModeResponse
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

(defclass kortex_driver::SetControllerConfigurationMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetControllerConfigurationMode :md5sum-) "a1e10ce64919ae3ba0f5b2193b693ecb")
(setf (get kortex_driver::SetControllerConfigurationMode :datatype-) "kortex_driver/SetControllerConfigurationMode")
(setf (get kortex_driver::SetControllerConfigurationMode :request) kortex_driver::SetControllerConfigurationModeRequest)
(setf (get kortex_driver::SetControllerConfigurationMode :response) kortex_driver::SetControllerConfigurationModeResponse)

(defmethod kortex_driver::SetControllerConfigurationModeRequest
  (:response () (instance kortex_driver::SetControllerConfigurationModeResponse :init)))

(setf (get kortex_driver::SetControllerConfigurationModeRequest :md5sum-) "a1e10ce64919ae3ba0f5b2193b693ecb")
(setf (get kortex_driver::SetControllerConfigurationModeRequest :datatype-) "kortex_driver/SetControllerConfigurationModeRequest")
(setf (get kortex_driver::SetControllerConfigurationModeRequest :definition-)
      "ControllerConfigurationMode input

================================================================================
MSG: kortex_driver/ControllerConfigurationMode

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetControllerConfigurationModeResponse :md5sum-) "a1e10ce64919ae3ba0f5b2193b693ecb")
(setf (get kortex_driver::SetControllerConfigurationModeResponse :datatype-) "kortex_driver/SetControllerConfigurationModeResponse")
(setf (get kortex_driver::SetControllerConfigurationModeResponse :definition-)
      "ControllerConfigurationMode input

================================================================================
MSG: kortex_driver/ControllerConfigurationMode

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetControllerConfigurationMode "a1e10ce64919ae3ba0f5b2193b693ecb")


