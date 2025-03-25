;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationServoingModeTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationServoingModeTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSERVOINGMODETOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSERVOINGMODETOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSERVOINGMODETOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSERVOINGMODETOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONSERVOINGMODETOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONSERVOINGMODETOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationServoingModeTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationServoingModeTopicRequest
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

(defclass kortex_driver::OnNotificationServoingModeTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationServoingModeTopicResponse
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

(defclass kortex_driver::OnNotificationServoingModeTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationServoingModeTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationServoingModeTopic :datatype-) "kortex_driver/OnNotificationServoingModeTopic")
(setf (get kortex_driver::OnNotificationServoingModeTopic :request) kortex_driver::OnNotificationServoingModeTopicRequest)
(setf (get kortex_driver::OnNotificationServoingModeTopic :response) kortex_driver::OnNotificationServoingModeTopicResponse)

(defmethod kortex_driver::OnNotificationServoingModeTopicRequest
  (:response () (instance kortex_driver::OnNotificationServoingModeTopicResponse :init)))

(setf (get kortex_driver::OnNotificationServoingModeTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationServoingModeTopicRequest :datatype-) "kortex_driver/OnNotificationServoingModeTopicRequest")
(setf (get kortex_driver::OnNotificationServoingModeTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationServoingModeTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationServoingModeTopicResponse :datatype-) "kortex_driver/OnNotificationServoingModeTopicResponse")
(setf (get kortex_driver::OnNotificationServoingModeTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationServoingModeTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


