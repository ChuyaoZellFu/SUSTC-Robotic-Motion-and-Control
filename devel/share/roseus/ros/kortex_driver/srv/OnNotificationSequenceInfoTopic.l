;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationSequenceInfoTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationSequenceInfoTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSEQUENCEINFOTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSEQUENCEINFOTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSEQUENCEINFOTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSEQUENCEINFOTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSEQUENCEINFOTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSEQUENCEINFOTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationSequenceInfoTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationSequenceInfoTopicRequest
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

(defclass kortex_driver::OnNotificationSequenceInfoTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationSequenceInfoTopicResponse
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

(defclass kortex_driver::OnNotificationSequenceInfoTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationSequenceInfoTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationSequenceInfoTopic :datatype-) "kortex_driver/OnNotificationSequenceInfoTopic")
(setf (get kortex_driver::OnNotificationSequenceInfoTopic :request) kortex_driver::OnNotificationSequenceInfoTopicRequest)
(setf (get kortex_driver::OnNotificationSequenceInfoTopic :response) kortex_driver::OnNotificationSequenceInfoTopicResponse)

(defmethod kortex_driver::OnNotificationSequenceInfoTopicRequest
  (:response () (instance kortex_driver::OnNotificationSequenceInfoTopicResponse :init)))

(setf (get kortex_driver::OnNotificationSequenceInfoTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationSequenceInfoTopicRequest :datatype-) "kortex_driver/OnNotificationSequenceInfoTopicRequest")
(setf (get kortex_driver::OnNotificationSequenceInfoTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationSequenceInfoTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationSequenceInfoTopicResponse :datatype-) "kortex_driver/OnNotificationSequenceInfoTopicResponse")
(setf (get kortex_driver::OnNotificationSequenceInfoTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationSequenceInfoTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


