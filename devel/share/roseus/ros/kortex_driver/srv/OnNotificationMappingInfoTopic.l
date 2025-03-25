;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationMappingInfoTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationMappingInfoTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONMAPPINGINFOTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONMAPPINGINFOTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONMAPPINGINFOTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONMAPPINGINFOTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONMAPPINGINFOTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONMAPPINGINFOTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationMappingInfoTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationMappingInfoTopicRequest
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

(defclass kortex_driver::OnNotificationMappingInfoTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationMappingInfoTopicResponse
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

(defclass kortex_driver::OnNotificationMappingInfoTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationMappingInfoTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationMappingInfoTopic :datatype-) "kortex_driver/OnNotificationMappingInfoTopic")
(setf (get kortex_driver::OnNotificationMappingInfoTopic :request) kortex_driver::OnNotificationMappingInfoTopicRequest)
(setf (get kortex_driver::OnNotificationMappingInfoTopic :response) kortex_driver::OnNotificationMappingInfoTopicResponse)

(defmethod kortex_driver::OnNotificationMappingInfoTopicRequest
  (:response () (instance kortex_driver::OnNotificationMappingInfoTopicResponse :init)))

(setf (get kortex_driver::OnNotificationMappingInfoTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationMappingInfoTopicRequest :datatype-) "kortex_driver/OnNotificationMappingInfoTopicRequest")
(setf (get kortex_driver::OnNotificationMappingInfoTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationMappingInfoTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationMappingInfoTopicResponse :datatype-) "kortex_driver/OnNotificationMappingInfoTopicResponse")
(setf (get kortex_driver::OnNotificationMappingInfoTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationMappingInfoTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


