;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetControlLoopParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetControlLoopParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLOOPPARAMETERS")
  (make-package "KORTEX_DRIVER::SETCONTROLLOOPPARAMETERS"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLOOPPARAMETERSREQUEST")
  (make-package "KORTEX_DRIVER::SETCONTROLLOOPPARAMETERSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCONTROLLOOPPARAMETERSRESPONSE")
  (make-package "KORTEX_DRIVER::SETCONTROLLOOPPARAMETERSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetControlLoopParametersRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetControlLoopParametersRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControlLoopParameters :init))
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
    ;; kortex_driver/ControlLoopParameters _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControlLoopParameters _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControlLoopParameters _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetControlLoopParametersResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetControlLoopParametersResponse
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

(defclass kortex_driver::SetControlLoopParameters
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetControlLoopParameters :md5sum-) "233c2c0defb94e1f2d65e518d2df07b0")
(setf (get kortex_driver::SetControlLoopParameters :datatype-) "kortex_driver/SetControlLoopParameters")
(setf (get kortex_driver::SetControlLoopParameters :request) kortex_driver::SetControlLoopParametersRequest)
(setf (get kortex_driver::SetControlLoopParameters :response) kortex_driver::SetControlLoopParametersResponse)

(defmethod kortex_driver::SetControlLoopParametersRequest
  (:response () (instance kortex_driver::SetControlLoopParametersResponse :init)))

(setf (get kortex_driver::SetControlLoopParametersRequest :md5sum-) "233c2c0defb94e1f2d65e518d2df07b0")
(setf (get kortex_driver::SetControlLoopParametersRequest :datatype-) "kortex_driver/SetControlLoopParametersRequest")
(setf (get kortex_driver::SetControlLoopParametersRequest :definition-)
      "ControlLoopParameters input

================================================================================
MSG: kortex_driver/ControlLoopParameters

uint32 loop_selection
float32 error_saturation
float32 output_saturation
float32[] kAz
float32[] kBz
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetControlLoopParametersResponse :md5sum-) "233c2c0defb94e1f2d65e518d2df07b0")
(setf (get kortex_driver::SetControlLoopParametersResponse :datatype-) "kortex_driver/SetControlLoopParametersResponse")
(setf (get kortex_driver::SetControlLoopParametersResponse :definition-)
      "ControlLoopParameters input

================================================================================
MSG: kortex_driver/ControlLoopParameters

uint32 loop_selection
float32 error_saturation
float32 output_saturation
float32[] kAz
float32[] kBz
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetControlLoopParameters "233c2c0defb94e1f2d65e518d2df07b0")


