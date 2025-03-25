;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetSafetyErrorThreshold)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetSafetyErrorThreshold (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETSAFETYERRORTHRESHOLD")
  (make-package "KORTEX_DRIVER::SETSAFETYERRORTHRESHOLD"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYERRORTHRESHOLDREQUEST")
  (make-package "KORTEX_DRIVER::SETSAFETYERRORTHRESHOLDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYERRORTHRESHOLDRESPONSE")
  (make-package "KORTEX_DRIVER::SETSAFETYERRORTHRESHOLDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetSafetyErrorThresholdRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetSafetyErrorThresholdRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SafetyThreshold :init))
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
    ;; kortex_driver/SafetyThreshold _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyThreshold _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyThreshold _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetSafetyErrorThresholdResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetSafetyErrorThresholdResponse
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

(defclass kortex_driver::SetSafetyErrorThreshold
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetSafetyErrorThreshold :md5sum-) "0d6527f8c0e63583f63fafeb1405804a")
(setf (get kortex_driver::SetSafetyErrorThreshold :datatype-) "kortex_driver/SetSafetyErrorThreshold")
(setf (get kortex_driver::SetSafetyErrorThreshold :request) kortex_driver::SetSafetyErrorThresholdRequest)
(setf (get kortex_driver::SetSafetyErrorThreshold :response) kortex_driver::SetSafetyErrorThresholdResponse)

(defmethod kortex_driver::SetSafetyErrorThresholdRequest
  (:response () (instance kortex_driver::SetSafetyErrorThresholdResponse :init)))

(setf (get kortex_driver::SetSafetyErrorThresholdRequest :md5sum-) "0d6527f8c0e63583f63fafeb1405804a")
(setf (get kortex_driver::SetSafetyErrorThresholdRequest :datatype-) "kortex_driver/SetSafetyErrorThresholdRequest")
(setf (get kortex_driver::SetSafetyErrorThresholdRequest :definition-)
      "SafetyThreshold input

================================================================================
MSG: kortex_driver/SafetyThreshold

SafetyHandle handle
float32 value
================================================================================
MSG: kortex_driver/SafetyHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetSafetyErrorThresholdResponse :md5sum-) "0d6527f8c0e63583f63fafeb1405804a")
(setf (get kortex_driver::SetSafetyErrorThresholdResponse :datatype-) "kortex_driver/SetSafetyErrorThresholdResponse")
(setf (get kortex_driver::SetSafetyErrorThresholdResponse :definition-)
      "SafetyThreshold input

================================================================================
MSG: kortex_driver/SafetyThreshold

SafetyHandle handle
float32 value
================================================================================
MSG: kortex_driver/SafetyHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetSafetyErrorThreshold "0d6527f8c0e63583f63fafeb1405804a")


