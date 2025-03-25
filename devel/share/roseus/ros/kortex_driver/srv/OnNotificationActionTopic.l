;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationActionTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationActionTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONACTIONTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONACTIONTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONACTIONTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONACTIONTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONACTIONTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONACTIONTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationActionTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationActionTopicRequest
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

(defclass kortex_driver::OnNotificationActionTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationActionTopicResponse
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

(defclass kortex_driver::OnNotificationActionTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationActionTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationActionTopic :datatype-) "kortex_driver/OnNotificationActionTopic")
(setf (get kortex_driver::OnNotificationActionTopic :request) kortex_driver::OnNotificationActionTopicRequest)
(setf (get kortex_driver::OnNotificationActionTopic :response) kortex_driver::OnNotificationActionTopicResponse)

(defmethod kortex_driver::OnNotificationActionTopicRequest
  (:response () (instance kortex_driver::OnNotificationActionTopicResponse :init)))

(setf (get kortex_driver::OnNotificationActionTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationActionTopicRequest :datatype-) "kortex_driver/OnNotificationActionTopicRequest")
(setf (get kortex_driver::OnNotificationActionTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationActionTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationActionTopicResponse :datatype-) "kortex_driver/OnNotificationActionTopicResponse")
(setf (get kortex_driver::OnNotificationActionTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationActionTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


