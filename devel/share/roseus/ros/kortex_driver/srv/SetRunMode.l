;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetRunMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetRunMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETRUNMODE")
  (make-package "KORTEX_DRIVER::SETRUNMODE"))
(unless (find-package "KORTEX_DRIVER::SETRUNMODEREQUEST")
  (make-package "KORTEX_DRIVER::SETRUNMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETRUNMODERESPONSE")
  (make-package "KORTEX_DRIVER::SETRUNMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetRunModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetRunModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::RunMode :init))
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
    ;; kortex_driver/RunMode _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/RunMode _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/RunMode _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetRunModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetRunModeResponse
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

(defclass kortex_driver::SetRunMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetRunMode :md5sum-) "9b8a67627c37962f119f97d74affc2cb")
(setf (get kortex_driver::SetRunMode :datatype-) "kortex_driver/SetRunMode")
(setf (get kortex_driver::SetRunMode :request) kortex_driver::SetRunModeRequest)
(setf (get kortex_driver::SetRunMode :response) kortex_driver::SetRunModeResponse)

(defmethod kortex_driver::SetRunModeRequest
  (:response () (instance kortex_driver::SetRunModeResponse :init)))

(setf (get kortex_driver::SetRunModeRequest :md5sum-) "9b8a67627c37962f119f97d74affc2cb")
(setf (get kortex_driver::SetRunModeRequest :datatype-) "kortex_driver/SetRunModeRequest")
(setf (get kortex_driver::SetRunModeRequest :definition-)
      "RunMode input

================================================================================
MSG: kortex_driver/RunMode

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetRunModeResponse :md5sum-) "9b8a67627c37962f119f97d74affc2cb")
(setf (get kortex_driver::SetRunModeResponse :datatype-) "kortex_driver/SetRunModeResponse")
(setf (get kortex_driver::SetRunModeResponse :definition-)
      "RunMode input

================================================================================
MSG: kortex_driver/RunMode

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetRunMode "9b8a67627c37962f119f97d74affc2cb")


