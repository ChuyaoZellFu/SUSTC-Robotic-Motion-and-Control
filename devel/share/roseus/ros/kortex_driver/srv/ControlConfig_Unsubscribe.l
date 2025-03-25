;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlConfig_Unsubscribe)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlConfig_Unsubscribe (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIG_UNSUBSCRIBE")
  (make-package "KORTEX_DRIVER::CONTROLCONFIG_UNSUBSCRIBE"))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIG_UNSUBSCRIBEREQUEST")
  (make-package "KORTEX_DRIVER::CONTROLCONFIG_UNSUBSCRIBEREQUEST"))
(unless (find-package "KORTEX_DRIVER::CONTROLCONFIG_UNSUBSCRIBERESPONSE")
  (make-package "KORTEX_DRIVER::CONTROLCONFIG_UNSUBSCRIBERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ControlConfig_UnsubscribeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ControlConfig_UnsubscribeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::NotificationHandle :init))
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
    ;; kortex_driver/NotificationHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/NotificationHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/NotificationHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ControlConfig_UnsubscribeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ControlConfig_UnsubscribeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ControlConfig_Unsubscribe
  :super ros::object
  :slots ())

(setf (get kortex_driver::ControlConfig_Unsubscribe :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::ControlConfig_Unsubscribe :datatype-) "kortex_driver/ControlConfig_Unsubscribe")
(setf (get kortex_driver::ControlConfig_Unsubscribe :request) kortex_driver::ControlConfig_UnsubscribeRequest)
(setf (get kortex_driver::ControlConfig_Unsubscribe :response) kortex_driver::ControlConfig_UnsubscribeResponse)

(defmethod kortex_driver::ControlConfig_UnsubscribeRequest
  (:response () (instance kortex_driver::ControlConfig_UnsubscribeResponse :init)))

(setf (get kortex_driver::ControlConfig_UnsubscribeRequest :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::ControlConfig_UnsubscribeRequest :datatype-) "kortex_driver/ControlConfig_UnsubscribeRequest")
(setf (get kortex_driver::ControlConfig_UnsubscribeRequest :definition-)
      "NotificationHandle input

================================================================================
MSG: kortex_driver/NotificationHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ControlConfig_UnsubscribeResponse :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::ControlConfig_UnsubscribeResponse :datatype-) "kortex_driver/ControlConfig_UnsubscribeResponse")
(setf (get kortex_driver::ControlConfig_UnsubscribeResponse :definition-)
      "NotificationHandle input

================================================================================
MSG: kortex_driver/NotificationHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ControlConfig_Unsubscribe "7960dab80443332660b3bb4bf774c9ab")


