;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetSafetyWarningThreshold)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetSafetyWarningThreshold (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETSAFETYWARNINGTHRESHOLD")
  (make-package "KORTEX_DRIVER::SETSAFETYWARNINGTHRESHOLD"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYWARNINGTHRESHOLDREQUEST")
  (make-package "KORTEX_DRIVER::SETSAFETYWARNINGTHRESHOLDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYWARNINGTHRESHOLDRESPONSE")
  (make-package "KORTEX_DRIVER::SETSAFETYWARNINGTHRESHOLDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetSafetyWarningThresholdRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetSafetyWarningThresholdRequest
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

(defclass kortex_driver::SetSafetyWarningThresholdResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetSafetyWarningThresholdResponse
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

(defclass kortex_driver::SetSafetyWarningThreshold
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetSafetyWarningThreshold :md5sum-) "0d6527f8c0e63583f63fafeb1405804a")
(setf (get kortex_driver::SetSafetyWarningThreshold :datatype-) "kortex_driver/SetSafetyWarningThreshold")
(setf (get kortex_driver::SetSafetyWarningThreshold :request) kortex_driver::SetSafetyWarningThresholdRequest)
(setf (get kortex_driver::SetSafetyWarningThreshold :response) kortex_driver::SetSafetyWarningThresholdResponse)

(defmethod kortex_driver::SetSafetyWarningThresholdRequest
  (:response () (instance kortex_driver::SetSafetyWarningThresholdResponse :init)))

(setf (get kortex_driver::SetSafetyWarningThresholdRequest :md5sum-) "0d6527f8c0e63583f63fafeb1405804a")
(setf (get kortex_driver::SetSafetyWarningThresholdRequest :datatype-) "kortex_driver/SetSafetyWarningThresholdRequest")
(setf (get kortex_driver::SetSafetyWarningThresholdRequest :definition-)
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

(setf (get kortex_driver::SetSafetyWarningThresholdResponse :md5sum-) "0d6527f8c0e63583f63fafeb1405804a")
(setf (get kortex_driver::SetSafetyWarningThresholdResponse :datatype-) "kortex_driver/SetSafetyWarningThresholdResponse")
(setf (get kortex_driver::SetSafetyWarningThresholdResponse :definition-)
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



(provide :kortex_driver/SetSafetyWarningThreshold "0d6527f8c0e63583f63fafeb1405804a")


