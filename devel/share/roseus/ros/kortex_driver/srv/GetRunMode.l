;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetRunMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetRunMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETRUNMODE")
  (make-package "KORTEX_DRIVER::GETRUNMODE"))
(unless (find-package "KORTEX_DRIVER::GETRUNMODEREQUEST")
  (make-package "KORTEX_DRIVER::GETRUNMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETRUNMODERESPONSE")
  (make-package "KORTEX_DRIVER::GETRUNMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetRunModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetRunModeRequest
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

(defclass kortex_driver::GetRunModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetRunModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::RunMode :init))
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
    ;; kortex_driver/RunMode _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/RunMode _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/RunMode _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetRunMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetRunMode :md5sum-) "dd5c8711b1fd2894a1cd52ad67ed6ff2")
(setf (get kortex_driver::GetRunMode :datatype-) "kortex_driver/GetRunMode")
(setf (get kortex_driver::GetRunMode :request) kortex_driver::GetRunModeRequest)
(setf (get kortex_driver::GetRunMode :response) kortex_driver::GetRunModeResponse)

(defmethod kortex_driver::GetRunModeRequest
  (:response () (instance kortex_driver::GetRunModeResponse :init)))

(setf (get kortex_driver::GetRunModeRequest :md5sum-) "dd5c8711b1fd2894a1cd52ad67ed6ff2")
(setf (get kortex_driver::GetRunModeRequest :datatype-) "kortex_driver/GetRunModeRequest")
(setf (get kortex_driver::GetRunModeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
RunMode output

================================================================================
MSG: kortex_driver/RunMode

")

(setf (get kortex_driver::GetRunModeResponse :md5sum-) "dd5c8711b1fd2894a1cd52ad67ed6ff2")
(setf (get kortex_driver::GetRunModeResponse :datatype-) "kortex_driver/GetRunModeResponse")
(setf (get kortex_driver::GetRunModeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
RunMode output

================================================================================
MSG: kortex_driver/RunMode

")



(provide :kortex_driver/GetRunMode "dd5c8711b1fd2894a1cd52ad67ed6ff2")


