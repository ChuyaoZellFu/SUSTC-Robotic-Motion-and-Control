;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetCoggingFeedforwardMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetCoggingFeedforwardMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCOGGINGFEEDFORWARDMODE")
  (make-package "KORTEX_DRIVER::SETCOGGINGFEEDFORWARDMODE"))
(unless (find-package "KORTEX_DRIVER::SETCOGGINGFEEDFORWARDMODEREQUEST")
  (make-package "KORTEX_DRIVER::SETCOGGINGFEEDFORWARDMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCOGGINGFEEDFORWARDMODERESPONSE")
  (make-package "KORTEX_DRIVER::SETCOGGINGFEEDFORWARDMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetCoggingFeedforwardModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetCoggingFeedforwardModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::CoggingFeedforwardModeInformation :init))
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
    ;; kortex_driver/CoggingFeedforwardModeInformation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CoggingFeedforwardModeInformation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CoggingFeedforwardModeInformation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetCoggingFeedforwardModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetCoggingFeedforwardModeResponse
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

(defclass kortex_driver::SetCoggingFeedforwardMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetCoggingFeedforwardMode :md5sum-) "156e5a43117ffd12af553796e457d579")
(setf (get kortex_driver::SetCoggingFeedforwardMode :datatype-) "kortex_driver/SetCoggingFeedforwardMode")
(setf (get kortex_driver::SetCoggingFeedforwardMode :request) kortex_driver::SetCoggingFeedforwardModeRequest)
(setf (get kortex_driver::SetCoggingFeedforwardMode :response) kortex_driver::SetCoggingFeedforwardModeResponse)

(defmethod kortex_driver::SetCoggingFeedforwardModeRequest
  (:response () (instance kortex_driver::SetCoggingFeedforwardModeResponse :init)))

(setf (get kortex_driver::SetCoggingFeedforwardModeRequest :md5sum-) "156e5a43117ffd12af553796e457d579")
(setf (get kortex_driver::SetCoggingFeedforwardModeRequest :datatype-) "kortex_driver/SetCoggingFeedforwardModeRequest")
(setf (get kortex_driver::SetCoggingFeedforwardModeRequest :definition-)
      "CoggingFeedforwardModeInformation input

================================================================================
MSG: kortex_driver/CoggingFeedforwardModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetCoggingFeedforwardModeResponse :md5sum-) "156e5a43117ffd12af553796e457d579")
(setf (get kortex_driver::SetCoggingFeedforwardModeResponse :datatype-) "kortex_driver/SetCoggingFeedforwardModeResponse")
(setf (get kortex_driver::SetCoggingFeedforwardModeResponse :definition-)
      "CoggingFeedforwardModeInformation input

================================================================================
MSG: kortex_driver/CoggingFeedforwardModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetCoggingFeedforwardMode "156e5a43117ffd12af553796e457d579")


