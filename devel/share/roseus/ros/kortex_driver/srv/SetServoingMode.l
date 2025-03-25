;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetServoingMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetServoingMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETSERVOINGMODE")
  (make-package "KORTEX_DRIVER::SETSERVOINGMODE"))
(unless (find-package "KORTEX_DRIVER::SETSERVOINGMODEREQUEST")
  (make-package "KORTEX_DRIVER::SETSERVOINGMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETSERVOINGMODERESPONSE")
  (make-package "KORTEX_DRIVER::SETSERVOINGMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetServoingModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetServoingModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ServoingModeInformation :init))
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
    ;; kortex_driver/ServoingModeInformation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ServoingModeInformation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ServoingModeInformation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetServoingModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetServoingModeResponse
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

(defclass kortex_driver::SetServoingMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetServoingMode :md5sum-) "a5076ba162fd0403b8672e41a673c035")
(setf (get kortex_driver::SetServoingMode :datatype-) "kortex_driver/SetServoingMode")
(setf (get kortex_driver::SetServoingMode :request) kortex_driver::SetServoingModeRequest)
(setf (get kortex_driver::SetServoingMode :response) kortex_driver::SetServoingModeResponse)

(defmethod kortex_driver::SetServoingModeRequest
  (:response () (instance kortex_driver::SetServoingModeResponse :init)))

(setf (get kortex_driver::SetServoingModeRequest :md5sum-) "a5076ba162fd0403b8672e41a673c035")
(setf (get kortex_driver::SetServoingModeRequest :datatype-) "kortex_driver/SetServoingModeRequest")
(setf (get kortex_driver::SetServoingModeRequest :definition-)
      "ServoingModeInformation input

================================================================================
MSG: kortex_driver/ServoingModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetServoingModeResponse :md5sum-) "a5076ba162fd0403b8672e41a673c035")
(setf (get kortex_driver::SetServoingModeResponse :datatype-) "kortex_driver/SetServoingModeResponse")
(setf (get kortex_driver::SetServoingModeResponse :definition-)
      "ServoingModeInformation input

================================================================================
MSG: kortex_driver/ServoingModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetServoingMode "a5076ba162fd0403b8672e41a673c035")


