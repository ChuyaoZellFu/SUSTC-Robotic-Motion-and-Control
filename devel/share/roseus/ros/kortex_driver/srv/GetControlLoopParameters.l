;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetControlLoopParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetControlLoopParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLOOPPARAMETERS")
  (make-package "KORTEX_DRIVER::GETCONTROLLOOPPARAMETERS"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLOOPPARAMETERSREQUEST")
  (make-package "KORTEX_DRIVER::GETCONTROLLOOPPARAMETERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLOOPPARAMETERSRESPONSE")
  (make-package "KORTEX_DRIVER::GETCONTROLLOOPPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetControlLoopParametersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetControlLoopParametersRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::LoopSelection :init))
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
    ;; kortex_driver/LoopSelection _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/LoopSelection _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/LoopSelection _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetControlLoopParametersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetControlLoopParametersResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ControlLoopParameters :init))
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
    ;; kortex_driver/ControlLoopParameters _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControlLoopParameters _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControlLoopParameters _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetControlLoopParameters
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetControlLoopParameters :md5sum-) "fe660929cab3528eb46e3ce651e5ce58")
(setf (get kortex_driver::GetControlLoopParameters :datatype-) "kortex_driver/GetControlLoopParameters")
(setf (get kortex_driver::GetControlLoopParameters :request) kortex_driver::GetControlLoopParametersRequest)
(setf (get kortex_driver::GetControlLoopParameters :response) kortex_driver::GetControlLoopParametersResponse)

(defmethod kortex_driver::GetControlLoopParametersRequest
  (:response () (instance kortex_driver::GetControlLoopParametersResponse :init)))

(setf (get kortex_driver::GetControlLoopParametersRequest :md5sum-) "fe660929cab3528eb46e3ce651e5ce58")
(setf (get kortex_driver::GetControlLoopParametersRequest :datatype-) "kortex_driver/GetControlLoopParametersRequest")
(setf (get kortex_driver::GetControlLoopParametersRequest :definition-)
      "LoopSelection input

================================================================================
MSG: kortex_driver/LoopSelection

---
ControlLoopParameters output

================================================================================
MSG: kortex_driver/ControlLoopParameters

uint32 loop_selection
float32 error_saturation
float32 output_saturation
float32[] kAz
float32[] kBz
")

(setf (get kortex_driver::GetControlLoopParametersResponse :md5sum-) "fe660929cab3528eb46e3ce651e5ce58")
(setf (get kortex_driver::GetControlLoopParametersResponse :datatype-) "kortex_driver/GetControlLoopParametersResponse")
(setf (get kortex_driver::GetControlLoopParametersResponse :definition-)
      "LoopSelection input

================================================================================
MSG: kortex_driver/LoopSelection

---
ControlLoopParameters output

================================================================================
MSG: kortex_driver/ControlLoopParameters

uint32 loop_selection
float32 error_saturation
float32 output_saturation
float32[] kAz
float32[] kBz
")



(provide :kortex_driver/GetControlLoopParameters "fe660929cab3528eb46e3ce651e5ce58")


