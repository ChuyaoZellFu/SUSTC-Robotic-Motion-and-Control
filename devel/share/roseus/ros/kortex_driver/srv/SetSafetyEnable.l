;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetSafetyEnable)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetSafetyEnable (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETSAFETYENABLE")
  (make-package "KORTEX_DRIVER::SETSAFETYENABLE"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYENABLEREQUEST")
  (make-package "KORTEX_DRIVER::SETSAFETYENABLEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETSAFETYENABLERESPONSE")
  (make-package "KORTEX_DRIVER::SETSAFETYENABLERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetSafetyEnableRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetSafetyEnableRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SafetyEnable :init))
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
    ;; kortex_driver/SafetyEnable _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyEnable _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyEnable _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetSafetyEnableResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetSafetyEnableResponse
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

(defclass kortex_driver::SetSafetyEnable
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetSafetyEnable :md5sum-) "81bf2032f72f340390e3e57ea5d159db")
(setf (get kortex_driver::SetSafetyEnable :datatype-) "kortex_driver/SetSafetyEnable")
(setf (get kortex_driver::SetSafetyEnable :request) kortex_driver::SetSafetyEnableRequest)
(setf (get kortex_driver::SetSafetyEnable :response) kortex_driver::SetSafetyEnableResponse)

(defmethod kortex_driver::SetSafetyEnableRequest
  (:response () (instance kortex_driver::SetSafetyEnableResponse :init)))

(setf (get kortex_driver::SetSafetyEnableRequest :md5sum-) "81bf2032f72f340390e3e57ea5d159db")
(setf (get kortex_driver::SetSafetyEnableRequest :datatype-) "kortex_driver/SetSafetyEnableRequest")
(setf (get kortex_driver::SetSafetyEnableRequest :definition-)
      "SafetyEnable input

================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
bool enable
================================================================================
MSG: kortex_driver/SafetyHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetSafetyEnableResponse :md5sum-) "81bf2032f72f340390e3e57ea5d159db")
(setf (get kortex_driver::SetSafetyEnableResponse :datatype-) "kortex_driver/SetSafetyEnableResponse")
(setf (get kortex_driver::SetSafetyEnableResponse :definition-)
      "SafetyEnable input

================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
bool enable
================================================================================
MSG: kortex_driver/SafetyHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetSafetyEnable "81bf2032f72f340390e3e57ea5d159db")


