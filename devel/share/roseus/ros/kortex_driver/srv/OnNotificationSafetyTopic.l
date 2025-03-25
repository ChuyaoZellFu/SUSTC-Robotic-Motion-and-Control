;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationSafetyTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationSafetyTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSAFETYTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSAFETYTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSAFETYTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSAFETYTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSAFETYTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSAFETYTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationSafetyTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationSafetyTopicRequest
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

(defclass kortex_driver::OnNotificationSafetyTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationSafetyTopicResponse
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

(defclass kortex_driver::OnNotificationSafetyTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationSafetyTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationSafetyTopic :datatype-) "kortex_driver/OnNotificationSafetyTopic")
(setf (get kortex_driver::OnNotificationSafetyTopic :request) kortex_driver::OnNotificationSafetyTopicRequest)
(setf (get kortex_driver::OnNotificationSafetyTopic :response) kortex_driver::OnNotificationSafetyTopicResponse)

(defmethod kortex_driver::OnNotificationSafetyTopicRequest
  (:response () (instance kortex_driver::OnNotificationSafetyTopicResponse :init)))

(setf (get kortex_driver::OnNotificationSafetyTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationSafetyTopicRequest :datatype-) "kortex_driver/OnNotificationSafetyTopicRequest")
(setf (get kortex_driver::OnNotificationSafetyTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationSafetyTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationSafetyTopicResponse :datatype-) "kortex_driver/OnNotificationSafetyTopicResponse")
(setf (get kortex_driver::OnNotificationSafetyTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationSafetyTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


