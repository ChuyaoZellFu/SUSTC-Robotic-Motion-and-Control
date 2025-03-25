;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationFactoryTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationFactoryTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONFACTORYTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONFACTORYTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONFACTORYTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONFACTORYTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONFACTORYTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONFACTORYTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationFactoryTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationFactoryTopicRequest
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

(defclass kortex_driver::OnNotificationFactoryTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationFactoryTopicResponse
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

(defclass kortex_driver::OnNotificationFactoryTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationFactoryTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationFactoryTopic :datatype-) "kortex_driver/OnNotificationFactoryTopic")
(setf (get kortex_driver::OnNotificationFactoryTopic :request) kortex_driver::OnNotificationFactoryTopicRequest)
(setf (get kortex_driver::OnNotificationFactoryTopic :response) kortex_driver::OnNotificationFactoryTopicResponse)

(defmethod kortex_driver::OnNotificationFactoryTopicRequest
  (:response () (instance kortex_driver::OnNotificationFactoryTopicResponse :init)))

(setf (get kortex_driver::OnNotificationFactoryTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationFactoryTopicRequest :datatype-) "kortex_driver/OnNotificationFactoryTopicRequest")
(setf (get kortex_driver::OnNotificationFactoryTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationFactoryTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationFactoryTopicResponse :datatype-) "kortex_driver/OnNotificationFactoryTopicResponse")
(setf (get kortex_driver::OnNotificationFactoryTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationFactoryTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


