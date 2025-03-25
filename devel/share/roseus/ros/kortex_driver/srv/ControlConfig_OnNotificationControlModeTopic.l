;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlConfig_OnNotificationControlModeTopic)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlConfig_OnNotificationControlModeTopic (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIG_ONNOTIFICATIONCONTROLMODETOPIC")
  (make-package "KORTEX_DRIVER::CONTROLCONFIG_ONNOTIFICATIONCONTROLMODETOPIC"))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIG_ONNOTIFICATIONCONTROLMODETOPICREQUEST")
  (make-package "KORTEX_DRIVER::CONTROLCONFIG_ONNOTIFICATIONCONTROLMODETOPICREQUEST"))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIG_ONNOTIFICATIONCONTROLMODETOPICRESPONSE")
  (make-package "KORTEX_DRIVER::CONTROLCONFIG_ONNOTIFICATIONCONTROLMODETOPICRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ControlConfig_OnNotificationControlModeTopicRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ControlConfig_OnNotificationControlModeTopicRequest
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

(defclass kortex_driver::ControlConfig_OnNotificationControlModeTopicResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ControlConfig_OnNotificationControlModeTopicResponse
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

(defclass kortex_driver::ControlConfig_OnNotificationControlModeTopic
  :super ros::object
  :slots ())

(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopic :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopic :datatype-) "kortex_driver/ControlConfig_OnNotificationControlModeTopic")
(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopic :request) kortex_driver::ControlConfig_OnNotificationControlModeTopicRequest)
(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopic :response) kortex_driver::ControlConfig_OnNotificationControlModeTopicResponse)

(defmethod kortex_driver::ControlConfig_OnNotificationControlModeTopicRequest
  (:response () (instance kortex_driver::ControlConfig_OnNotificationControlModeTopicResponse :init)))

(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopicRequest :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopicRequest :datatype-) "kortex_driver/ControlConfig_OnNotificationControlModeTopicRequest")
(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopicRequest :definition-)
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

(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopicResponse :md5sum-) "6fefdd07c6cb63a94f7b48e7e07e815b")
(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopicResponse :datatype-) "kortex_driver/ControlConfig_OnNotificationControlModeTopicResponse")
(setf (get kortex_driver::ControlConfig_OnNotificationControlModeTopicResponse :definition-)
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



(provide :kortex_driver/ControlConfig_OnNotificationControlModeTopic "6fefdd07c6cb63a94f7b48e7e07e815b")


