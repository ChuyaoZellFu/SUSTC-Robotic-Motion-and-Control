;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationConfigurationChangeTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationConfigurationChangeTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONFIGURATIONCHANGETOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONFIGURATIONCHANGETOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONFIGURATIONCHANGETOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONFIGURATIONCHANGETOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONFIGURATIONCHANGETOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONFIGURATIONCHANGETOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationConfigurationChangeTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationConfigurationChangeTopicRequest
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

(defclass kortex_driver::OnNotificationConfigurationChangeTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationConfigurationChangeTopicResponse
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

(defclass kortex_driver::OnNotificationConfigurationChangeTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationConfigurationChangeTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationConfigurationChangeTopic :datatype-) "kortex_driver/OnNotificationConfigurationChangeTopic")
(setf (get kortex_driver::OnNotificationConfigurationChangeTopic :request) kortex_driver::OnNotificationConfigurationChangeTopicRequest)
(setf (get kortex_driver::OnNotificationConfigurationChangeTopic :response) kortex_driver::OnNotificationConfigurationChangeTopicResponse)

(defmethod kortex_driver::OnNotificationConfigurationChangeTopicRequest
  (:response () (instance kortex_driver::OnNotificationConfigurationChangeTopicResponse :init)))

(setf (get kortex_driver::OnNotificationConfigurationChangeTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationConfigurationChangeTopicRequest :datatype-) "kortex_driver/OnNotificationConfigurationChangeTopicRequest")
(setf (get kortex_driver::OnNotificationConfigurationChangeTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationConfigurationChangeTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationConfigurationChangeTopicResponse :datatype-) "kortex_driver/OnNotificationConfigurationChangeTopicResponse")
(setf (get kortex_driver::OnNotificationConfigurationChangeTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationConfigurationChangeTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


