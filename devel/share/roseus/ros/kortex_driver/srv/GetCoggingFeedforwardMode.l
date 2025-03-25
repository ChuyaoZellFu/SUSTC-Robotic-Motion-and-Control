;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetCoggingFeedforwardMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetCoggingFeedforwardMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCOGGINGFEEDFORWARDMODE")
  (make-package "KORTEX_DRIVER::GETCOGGINGFEEDFORWARDMODE"))
(unless (find-package "KORTEX_DRIVER::GETCOGGINGFEEDFORWARDMODEREQUEST")
  (make-package "KORTEX_DRIVER::GETCOGGINGFEEDFORWARDMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCOGGINGFEEDFORWARDMODERESPONSE")
  (make-package "KORTEX_DRIVER::GETCOGGINGFEEDFORWARDMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetCoggingFeedforwardModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetCoggingFeedforwardModeRequest
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

(defclass kortex_driver::GetCoggingFeedforwardModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetCoggingFeedforwardModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CoggingFeedforwardModeInformation :init))
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
    ;; kortex_driver/CoggingFeedforwardModeInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CoggingFeedforwardModeInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CoggingFeedforwardModeInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetCoggingFeedforwardMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetCoggingFeedforwardMode :md5sum-) "54f13ab10b17b140d71d14c7e922bc7e")
(setf (get kortex_driver::GetCoggingFeedforwardMode :datatype-) "kortex_driver/GetCoggingFeedforwardMode")
(setf (get kortex_driver::GetCoggingFeedforwardMode :request) kortex_driver::GetCoggingFeedforwardModeRequest)
(setf (get kortex_driver::GetCoggingFeedforwardMode :response) kortex_driver::GetCoggingFeedforwardModeResponse)

(defmethod kortex_driver::GetCoggingFeedforwardModeRequest
  (:response () (instance kortex_driver::GetCoggingFeedforwardModeResponse :init)))

(setf (get kortex_driver::GetCoggingFeedforwardModeRequest :md5sum-) "54f13ab10b17b140d71d14c7e922bc7e")
(setf (get kortex_driver::GetCoggingFeedforwardModeRequest :datatype-) "kortex_driver/GetCoggingFeedforwardModeRequest")
(setf (get kortex_driver::GetCoggingFeedforwardModeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CoggingFeedforwardModeInformation output

================================================================================
MSG: kortex_driver/CoggingFeedforwardModeInformation

")

(setf (get kortex_driver::GetCoggingFeedforwardModeResponse :md5sum-) "54f13ab10b17b140d71d14c7e922bc7e")
(setf (get kortex_driver::GetCoggingFeedforwardModeResponse :datatype-) "kortex_driver/GetCoggingFeedforwardModeResponse")
(setf (get kortex_driver::GetCoggingFeedforwardModeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CoggingFeedforwardModeInformation output

================================================================================
MSG: kortex_driver/CoggingFeedforwardModeInformation

")



(provide :kortex_driver/GetCoggingFeedforwardMode "54f13ab10b17b140d71d14c7e922bc7e")


