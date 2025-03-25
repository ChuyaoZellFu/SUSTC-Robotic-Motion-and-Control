;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationNetworkTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationNetworkTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONNETWORKTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONNETWORKTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONNETWORKTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONNETWORKTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONNETWORKTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONNETWORKTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationNetworkTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationNetworkTopicRequest
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

(defclass kortex_driver::OnNotificationNetworkTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationNetworkTopicResponse
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

(defclass kortex_driver::OnNotificationNetworkTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationNetworkTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationNetworkTopic :datatype-) "kortex_driver/OnNotificationNetworkTopic")
(setf (get kortex_driver::OnNotificationNetworkTopic :request) kortex_driver::OnNotificationNetworkTopicRequest)
(setf (get kortex_driver::OnNotificationNetworkTopic :response) kortex_driver::OnNotificationNetworkTopicResponse)

(defmethod kortex_driver::OnNotificationNetworkTopicRequest
  (:response () (instance kortex_driver::OnNotificationNetworkTopicResponse :init)))

(setf (get kortex_driver::OnNotificationNetworkTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationNetworkTopicRequest :datatype-) "kortex_driver/OnNotificationNetworkTopicRequest")
(setf (get kortex_driver::OnNotificationNetworkTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationNetworkTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationNetworkTopicResponse :datatype-) "kortex_driver/OnNotificationNetworkTopicResponse")
(setf (get kortex_driver::OnNotificationNetworkTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationNetworkTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


