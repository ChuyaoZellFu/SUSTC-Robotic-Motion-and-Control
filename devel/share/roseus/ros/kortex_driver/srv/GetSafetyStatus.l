;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetSafetyStatus)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetSafetyStatus (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSAFETYSTATUS")
  (make-package "KORTEX_DRIVER::GETSAFETYSTATUS"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYSTATUSREQUEST")
  (make-package "KORTEX_DRIVER::GETSAFETYSTATUSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYSTATUSRESPONSE")
  (make-package "KORTEX_DRIVER::GETSAFETYSTATUSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetSafetyStatusRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetSafetyStatusRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SafetyHandle :init))
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
    ;; kortex_driver/SafetyHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSafetyStatusResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetSafetyStatusResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SafetyStatus :init))
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
    ;; kortex_driver/SafetyStatus _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyStatus _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyStatus _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSafetyStatus
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetSafetyStatus :md5sum-) "0b50aa56e0379c3a0c319bd4962efe2e")
(setf (get kortex_driver::GetSafetyStatus :datatype-) "kortex_driver/GetSafetyStatus")
(setf (get kortex_driver::GetSafetyStatus :request) kortex_driver::GetSafetyStatusRequest)
(setf (get kortex_driver::GetSafetyStatus :response) kortex_driver::GetSafetyStatusResponse)

(defmethod kortex_driver::GetSafetyStatusRequest
  (:response () (instance kortex_driver::GetSafetyStatusResponse :init)))

(setf (get kortex_driver::GetSafetyStatusRequest :md5sum-) "0b50aa56e0379c3a0c319bd4962efe2e")
(setf (get kortex_driver::GetSafetyStatusRequest :datatype-) "kortex_driver/GetSafetyStatusRequest")
(setf (get kortex_driver::GetSafetyStatusRequest :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyStatus output

================================================================================
MSG: kortex_driver/SafetyStatus

")

(setf (get kortex_driver::GetSafetyStatusResponse :md5sum-) "0b50aa56e0379c3a0c319bd4962efe2e")
(setf (get kortex_driver::GetSafetyStatusResponse :datatype-) "kortex_driver/GetSafetyStatusResponse")
(setf (get kortex_driver::GetSafetyStatusResponse :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyStatus output

================================================================================
MSG: kortex_driver/SafetyStatus

")



(provide :kortex_driver/GetSafetyStatus "0b50aa56e0379c3a0c319bd4962efe2e")


