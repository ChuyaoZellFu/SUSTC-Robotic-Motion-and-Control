;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationUserTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationUserTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONUSERTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONUSERTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONUSERTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONUSERTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONUSERTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONUSERTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationUserTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationUserTopicRequest
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

(defclass kortex_driver::OnNotificationUserTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationUserTopicResponse
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

(defclass kortex_driver::OnNotificationUserTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationUserTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationUserTopic :datatype-) "kortex_driver/OnNotificationUserTopic")
(setf (get kortex_driver::OnNotificationUserTopic :request) kortex_driver::OnNotificationUserTopicRequest)
(setf (get kortex_driver::OnNotificationUserTopic :response) kortex_driver::OnNotificationUserTopicResponse)

(defmethod kortex_driver::OnNotificationUserTopicRequest
  (:response () (instance kortex_driver::OnNotificationUserTopicResponse :init)))

(setf (get kortex_driver::OnNotificationUserTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationUserTopicRequest :datatype-) "kortex_driver/OnNotificationUserTopicRequest")
(setf (get kortex_driver::OnNotificationUserTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationUserTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationUserTopicResponse :datatype-) "kortex_driver/OnNotificationUserTopicResponse")
(setf (get kortex_driver::OnNotificationUserTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationUserTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


