;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetControlMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetControlMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCONTROLMODE")
  (make-package "KORTEX_DRIVER::SETCONTROLMODE"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLMODEREQUEST")
  (make-package "KORTEX_DRIVER::SETCONTROLMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLMODERESPONSE")
  (make-package "KORTEX_DRIVER::SETCONTROLMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetControlModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetControlModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ActuatorConfig_ControlModeInformation :init))
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
    ;; kortex_driver/ActuatorConfig_ControlModeInformation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ActuatorConfig_ControlModeInformation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ActuatorConfig_ControlModeInformation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetControlModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetControlModeResponse
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

(defclass kortex_driver::SetControlMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetControlMode :md5sum-) "cd48317b2c3b2a87721b906a23a31b2f")
(setf (get kortex_driver::SetControlMode :datatype-) "kortex_driver/SetControlMode")
(setf (get kortex_driver::SetControlMode :request) kortex_driver::SetControlModeRequest)
(setf (get kortex_driver::SetControlMode :response) kortex_driver::SetControlModeResponse)

(defmethod kortex_driver::SetControlModeRequest
  (:response () (instance kortex_driver::SetControlModeResponse :init)))

(setf (get kortex_driver::SetControlModeRequest :md5sum-) "cd48317b2c3b2a87721b906a23a31b2f")
(setf (get kortex_driver::SetControlModeRequest :datatype-) "kortex_driver/SetControlModeRequest")
(setf (get kortex_driver::SetControlModeRequest :definition-)
      "ActuatorConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ActuatorConfig_ControlModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetControlModeResponse :md5sum-) "cd48317b2c3b2a87721b906a23a31b2f")
(setf (get kortex_driver::SetControlModeResponse :datatype-) "kortex_driver/SetControlModeResponse")
(setf (get kortex_driver::SetControlModeResponse :definition-)
      "ActuatorConfig_ControlModeInformation input

================================================================================
MSG: kortex_driver/ActuatorConfig_ControlModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetControlMode "cd48317b2c3b2a87721b906a23a31b2f")


