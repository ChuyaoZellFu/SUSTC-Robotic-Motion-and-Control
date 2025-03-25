;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetServoingMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetServoingMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSERVOINGMODE")
  (make-package "KORTEX_DRIVER::GETSERVOINGMODE"))
(unless (find-package "KORTEX_DRIVER::GETSERVOINGMODEREQUEST")
  (make-package "KORTEX_DRIVER::GETSERVOINGMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSERVOINGMODERESPONSE")
  (make-package "KORTEX_DRIVER::GETSERVOINGMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetServoingModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetServoingModeRequest
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

(defclass kortex_driver::GetServoingModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetServoingModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ServoingModeInformation :init))
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
    ;; kortex_driver/ServoingModeInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ServoingModeInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ServoingModeInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetServoingMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetServoingMode :md5sum-) "530753c2004b520e20de588dc61fb225")
(setf (get kortex_driver::GetServoingMode :datatype-) "kortex_driver/GetServoingMode")
(setf (get kortex_driver::GetServoingMode :request) kortex_driver::GetServoingModeRequest)
(setf (get kortex_driver::GetServoingMode :response) kortex_driver::GetServoingModeResponse)

(defmethod kortex_driver::GetServoingModeRequest
  (:response () (instance kortex_driver::GetServoingModeResponse :init)))

(setf (get kortex_driver::GetServoingModeRequest :md5sum-) "530753c2004b520e20de588dc61fb225")
(setf (get kortex_driver::GetServoingModeRequest :datatype-) "kortex_driver/GetServoingModeRequest")
(setf (get kortex_driver::GetServoingModeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ServoingModeInformation output

================================================================================
MSG: kortex_driver/ServoingModeInformation

")

(setf (get kortex_driver::GetServoingModeResponse :md5sum-) "530753c2004b520e20de588dc61fb225")
(setf (get kortex_driver::GetServoingModeResponse :datatype-) "kortex_driver/GetServoingModeResponse")
(setf (get kortex_driver::GetServoingModeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ServoingModeInformation output

================================================================================
MSG: kortex_driver/ServoingModeInformation

")



(provide :kortex_driver/GetServoingMode "530753c2004b520e20de588dc61fb225")


