;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationVisionTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationVisionTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONVISIONTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONVISIONTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONVISIONTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONVISIONTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONVISIONTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONVISIONTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationVisionTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationVisionTopicRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::NotificationOptions :init))
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
    ;; kortex_driver/NotificationOptions _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/NotificationOptions _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/NotificationOptions _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::OnNotificationVisionTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationVisionTopicResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::NotificationHandle :init))
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
    ;; kortex_driver/NotificationHandle _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/NotificationHandle _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/NotificationHandle _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::OnNotificationVisionTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationVisionTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationVisionTopic :datatype-) "kortex_driver/OnNotificationVisionTopic")
(setf (get kortex_driver::OnNotificationVisionTopic :request) kortex_driver::OnNotificationVisionTopicRequest)
(setf (get kortex_driver::OnNotificationVisionTopic :response) kortex_driver::OnNotificationVisionTopicResponse)

(defmethod kortex_driver::OnNotificationVisionTopicRequest
  (:response () (instance kortex_driver::OnNotificationVisionTopicResponse :init)))

(setf (get kortex_driver::OnNotificationVisionTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationVisionTopicRequest :datatype-) "kortex_driver/OnNotificationVisionTopicRequest")
(setf (get kortex_driver::OnNotificationVisionTopicRequest :definition-)
      "NotificationOptions input

================================================================================
MSG: kortex_driver/NotificationOptions

uint32 type
uint32 rate_m_sec
---
NotificationHandle output

================================================================================
MSG: kortex_driver/NotificationHandle

")

(setf (get kortex_driver::OnNotificationVisionTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationVisionTopicResponse :datatype-) "kortex_driver/OnNotificationVisionTopicResponse")
(setf (get kortex_driver::OnNotificationVisionTopicResponse :definition-)
      "NotificationOptions input

================================================================================
MSG: kortex_driver/NotificationOptions

uint32 type
uint32 rate_m_sec
---
NotificationHandle output

================================================================================
MSG: kortex_driver/NotificationHandle

")



(provide :kortex_driver/OnNotificationVisionTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


