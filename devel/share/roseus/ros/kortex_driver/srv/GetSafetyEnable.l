;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetSafetyEnable)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetSafetyEnable (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSAFETYENABLE")
  (make-package "KORTEX_DRIVER::GETSAFETYENABLE"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYENABLEREQUEST")
  (make-package "KORTEX_DRIVER::GETSAFETYENABLEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSAFETYENABLERESPONSE")
  (make-package "KORTEX_DRIVER::GETSAFETYENABLERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetSafetyEnableRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetSafetyEnableRequest
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

(defclass kortex_driver::GetSafetyEnableResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetSafetyEnableResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SafetyEnable :init))
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
    ;; kortex_driver/SafetyEnable _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyEnable _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyEnable _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSafetyEnable
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetSafetyEnable :md5sum-) "d579190d4e318f6ca36b725f2e3b7043")
(setf (get kortex_driver::GetSafetyEnable :datatype-) "kortex_driver/GetSafetyEnable")
(setf (get kortex_driver::GetSafetyEnable :request) kortex_driver::GetSafetyEnableRequest)
(setf (get kortex_driver::GetSafetyEnable :response) kortex_driver::GetSafetyEnableResponse)

(defmethod kortex_driver::GetSafetyEnableRequest
  (:response () (instance kortex_driver::GetSafetyEnableResponse :init)))

(setf (get kortex_driver::GetSafetyEnableRequest :md5sum-) "d579190d4e318f6ca36b725f2e3b7043")
(setf (get kortex_driver::GetSafetyEnableRequest :datatype-) "kortex_driver/GetSafetyEnableRequest")
(setf (get kortex_driver::GetSafetyEnableRequest :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyEnable output

================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
bool enable
================================================================================
MSG: kortex_driver/SafetyHandle

")

(setf (get kortex_driver::GetSafetyEnableResponse :md5sum-) "d579190d4e318f6ca36b725f2e3b7043")
(setf (get kortex_driver::GetSafetyEnableResponse :datatype-) "kortex_driver/GetSafetyEnableResponse")
(setf (get kortex_driver::GetSafetyEnableResponse :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
SafetyEnable output

================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
bool enable
================================================================================
MSG: kortex_driver/SafetyHandle

")



(provide :kortex_driver/GetSafetyEnable "d579190d4e318f6ca36b725f2e3b7043")


