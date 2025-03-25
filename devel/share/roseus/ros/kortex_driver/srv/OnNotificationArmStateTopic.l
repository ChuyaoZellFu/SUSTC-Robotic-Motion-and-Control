;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationArmStateTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationArmStateTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONARMSTATETOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONARMSTATETOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONARMSTATETOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONARMSTATETOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONARMSTATETOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONARMSTATETOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationArmStateTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationArmStateTopicRequest
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

(defclass kortex_driver::OnNotificationArmStateTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationArmStateTopicResponse
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

(defclass kortex_driver::OnNotificationArmStateTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationArmStateTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationArmStateTopic :datatype-) "kortex_driver/OnNotificationArmStateTopic")
(setf (get kortex_driver::OnNotificationArmStateTopic :request) kortex_driver::OnNotificationArmStateTopicRequest)
(setf (get kortex_driver::OnNotificationArmStateTopic :response) kortex_driver::OnNotificationArmStateTopicResponse)

(defmethod kortex_driver::OnNotificationArmStateTopicRequest
  (:response () (instance kortex_driver::OnNotificationArmStateTopicResponse :init)))

(setf (get kortex_driver::OnNotificationArmStateTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationArmStateTopicRequest :datatype-) "kortex_driver/OnNotificationArmStateTopicRequest")
(setf (get kortex_driver::OnNotificationArmStateTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationArmStateTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationArmStateTopicResponse :datatype-) "kortex_driver/OnNotificationArmStateTopicResponse")
(setf (get kortex_driver::OnNotificationArmStateTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationArmStateTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


