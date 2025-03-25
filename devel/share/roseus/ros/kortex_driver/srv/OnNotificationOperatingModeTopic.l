;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationOperatingModeTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationOperatingModeTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONOPERATINGMODETOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONOPERATINGMODETOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONOPERATINGMODETOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONOPERATINGMODETOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONOPERATINGMODETOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONOPERATINGMODETOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationOperatingModeTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationOperatingModeTopicRequest
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

(defclass kortex_driver::OnNotificationOperatingModeTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationOperatingModeTopicResponse
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

(defclass kortex_driver::OnNotificationOperatingModeTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationOperatingModeTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationOperatingModeTopic :datatype-) "kortex_driver/OnNotificationOperatingModeTopic")
(setf (get kortex_driver::OnNotificationOperatingModeTopic :request) kortex_driver::OnNotificationOperatingModeTopicRequest)
(setf (get kortex_driver::OnNotificationOperatingModeTopic :response) kortex_driver::OnNotificationOperatingModeTopicResponse)

(defmethod kortex_driver::OnNotificationOperatingModeTopicRequest
  (:response () (instance kortex_driver::OnNotificationOperatingModeTopicResponse :init)))

(setf (get kortex_driver::OnNotificationOperatingModeTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationOperatingModeTopicRequest :datatype-) "kortex_driver/OnNotificationOperatingModeTopicRequest")
(setf (get kortex_driver::OnNotificationOperatingModeTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationOperatingModeTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationOperatingModeTopicResponse :datatype-) "kortex_driver/OnNotificationOperatingModeTopicResponse")
(setf (get kortex_driver::OnNotificationOperatingModeTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationOperatingModeTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


