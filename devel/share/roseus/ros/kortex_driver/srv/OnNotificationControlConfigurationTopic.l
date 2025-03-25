;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationControlConfigurationTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationControlConfigurationTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLCONFIGURATIONTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLCONFIGURATIONTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLCONFIGURATIONTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLCONFIGURATIONTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLCONFIGURATIONTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLCONFIGURATIONTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationControlConfigurationTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationControlConfigurationTopicRequest
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

(defclass kortex_driver::OnNotificationControlConfigurationTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationControlConfigurationTopicResponse
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

(defclass kortex_driver::OnNotificationControlConfigurationTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationControlConfigurationTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationControlConfigurationTopic :datatype-) "kortex_driver/OnNotificationControlConfigurationTopic")
(setf (get kortex_driver::OnNotificationControlConfigurationTopic :request) kortex_driver::OnNotificationControlConfigurationTopicRequest)
(setf (get kortex_driver::OnNotificationControlConfigurationTopic :response) kortex_driver::OnNotificationControlConfigurationTopicResponse)

(defmethod kortex_driver::OnNotificationControlConfigurationTopicRequest
  (:response () (instance kortex_driver::OnNotificationControlConfigurationTopicResponse :init)))

(setf (get kortex_driver::OnNotificationControlConfigurationTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationControlConfigurationTopicRequest :datatype-) "kortex_driver/OnNotificationControlConfigurationTopicRequest")
(setf (get kortex_driver::OnNotificationControlConfigurationTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationControlConfigurationTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationControlConfigurationTopicResponse :datatype-) "kortex_driver/OnNotificationControlConfigurationTopicResponse")
(setf (get kortex_driver::OnNotificationControlConfigurationTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationControlConfigurationTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


