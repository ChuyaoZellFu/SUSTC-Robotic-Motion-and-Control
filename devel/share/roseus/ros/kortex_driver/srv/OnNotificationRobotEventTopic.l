;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationRobotEventTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationRobotEventTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONROBOTEVENTTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONROBOTEVENTTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONROBOTEVENTTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONROBOTEVENTTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONROBOTEVENTTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONROBOTEVENTTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationRobotEventTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationRobotEventTopicRequest
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

(defclass kortex_driver::OnNotificationRobotEventTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationRobotEventTopicResponse
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

(defclass kortex_driver::OnNotificationRobotEventTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationRobotEventTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationRobotEventTopic :datatype-) "kortex_driver/OnNotificationRobotEventTopic")
(setf (get kortex_driver::OnNotificationRobotEventTopic :request) kortex_driver::OnNotificationRobotEventTopicRequest)
(setf (get kortex_driver::OnNotificationRobotEventTopic :response) kortex_driver::OnNotificationRobotEventTopicResponse)

(defmethod kortex_driver::OnNotificationRobotEventTopicRequest
  (:response () (instance kortex_driver::OnNotificationRobotEventTopicResponse :init)))

(setf (get kortex_driver::OnNotificationRobotEventTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationRobotEventTopicRequest :datatype-) "kortex_driver/OnNotificationRobotEventTopicRequest")
(setf (get kortex_driver::OnNotificationRobotEventTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationRobotEventTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationRobotEventTopicResponse :datatype-) "kortex_driver/OnNotificationRobotEventTopicResponse")
(setf (get kortex_driver::OnNotificationRobotEventTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationRobotEventTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


