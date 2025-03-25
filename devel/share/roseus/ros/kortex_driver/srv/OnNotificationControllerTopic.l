;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OnNotificationControllerTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OnNotificationControllerTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLLERTOPIC")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLLERTOPIC"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLLERTOPICREQUEST")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLLERTOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLLERTOPICRESPONSE")
  (make-package "KORTEX_DRIVER::ONNOTIFICATIONCONTROLLERTOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::OnNotificationControllerTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::OnNotificationControllerTopicRequest
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

(defclass kortex_driver::OnNotificationControllerTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::OnNotificationControllerTopicResponse
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

(defclass kortex_driver::OnNotificationControllerTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::OnNotificationControllerTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationControllerTopic :datatype-) "kortex_driver/OnNotificationControllerTopic")
(setf (get kortex_driver::OnNotificationControllerTopic :request) kortex_driver::OnNotificationControllerTopicRequest)
(setf (get kortex_driver::OnNotificationControllerTopic :response) kortex_driver::OnNotificationControllerTopicResponse)

(defmethod kortex_driver::OnNotificationControllerTopicRequest
  (:response () (instance kortex_driver::OnNotificationControllerTopicResponse :init)))

(setf (get kortex_driver::OnNotificationControllerTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationControllerTopicRequest :datatype-) "kortex_driver/OnNotificationControllerTopicRequest")
(setf (get kortex_driver::OnNotificationControllerTopicRequest :definition-)
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

(setf (get kortex_driver::OnNotificationControllerTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::OnNotificationControllerTopicResponse :datatype-) "kortex_driver/OnNotificationControllerTopicResponse")
(setf (get kortex_driver::OnNotificationControllerTopicResponse :definition-)
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



(provide :kortex_driver/OnNotificationControllerTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


