;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetControllerConfigurationMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetControllerConfigurationMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONMODE")
  (make-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONMODE"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONMODEREQUEST")
  (make-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONMODERESPONSE")
  (make-package "KORTEX_DRIVER::GETCONTROLLERCONFIGURATIONMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetControllerConfigurationModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetControllerConfigurationModeRequest
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

(defclass kortex_driver::GetControllerConfigurationModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetControllerConfigurationModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ControllerConfigurationMode :init))
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
    ;; kortex_driver/ControllerConfigurationMode _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerConfigurationMode _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerConfigurationMode _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetControllerConfigurationMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetControllerConfigurationMode :md5sum-) "f62e2e25b47b1394acdb18945114f7ef")
(setf (get kortex_driver::GetControllerConfigurationMode :datatype-) "kortex_driver/GetControllerConfigurationMode")
(setf (get kortex_driver::GetControllerConfigurationMode :request) kortex_driver::GetControllerConfigurationModeRequest)
(setf (get kortex_driver::GetControllerConfigurationMode :response) kortex_driver::GetControllerConfigurationModeResponse)

(defmethod kortex_driver::GetControllerConfigurationModeRequest
  (:response () (instance kortex_driver::GetControllerConfigurationModeResponse :init)))

(setf (get kortex_driver::GetControllerConfigurationModeRequest :md5sum-) "f62e2e25b47b1394acdb18945114f7ef")
(setf (get kortex_driver::GetControllerConfigurationModeRequest :datatype-) "kortex_driver/GetControllerConfigurationModeRequest")
(setf (get kortex_driver::GetControllerConfigurationModeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControllerConfigurationMode output

================================================================================
MSG: kortex_driver/ControllerConfigurationMode

")

(setf (get kortex_driver::GetControllerConfigurationModeResponse :md5sum-) "f62e2e25b47b1394acdb18945114f7ef")
(setf (get kortex_driver::GetControllerConfigurationModeResponse :datatype-) "kortex_driver/GetControllerConfigurationModeResponse")
(setf (get kortex_driver::GetControllerConfigurationModeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControllerConfigurationMode output

================================================================================
MSG: kortex_driver/ControllerConfigurationMode

")



(provide :kortex_driver/GetControllerConfigurationMode "f62e2e25b47b1394acdb18945114f7ef")


