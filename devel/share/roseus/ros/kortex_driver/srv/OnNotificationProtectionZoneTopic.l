;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationProtectionZoneTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationProtectionZoneTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONPROTECTIONZONETOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONPROTECTIONZONETOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONPROTECTIONZONETOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONPROTECTIONZONETOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONPROTECTIONZONETOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONPROTECTIONZONETOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationProtectionZoneTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationProtectionZoneTopicRequest
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

(defclass kortex_driver::OnNotificationProtectionZoneTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationProtectionZoneTopicResponse
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

(defclass kortex_driver::OnNotificationProtectionZoneTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationProtectionZoneTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationProtectionZoneTopic :datatype-) "kortex_driver/OnNotificationProtectionZoneTopic")
(setf (get kortex_driver::OnNotificationProtectionZoneTopic :request) kortex_driver::OnNotificationProtectionZoneTopicRequest)
(setf (get kortex_driver::OnNotificationProtectionZoneTopic :response) kortex_driver::OnNotificationProtectionZoneTopicResponse)

(defmethod kortex_driver::OnNotificationProtectionZoneTopicRequest
  (:response () (instance kortex_driver::OnNotificationProtectionZoneTopicResponse :init)))

(setf (get kortex_driver::OnNotificationProtectionZoneTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationProtectionZoneTopicRequest :datatype-) "kortex_driver/OnNotificationProtectionZoneTopicRequest")
(setf (get kortex_driver::OnNotificationProtectionZoneTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationProtectionZoneTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationProtectionZoneTopicResponse :datatype-) "kortex_driver/OnNotificationProtectionZoneTopicResponse")
(setf (get kortex_driver::OnNotificationProtectionZoneTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationProtectionZoneTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


