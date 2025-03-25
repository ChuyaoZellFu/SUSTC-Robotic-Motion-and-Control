;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetWifiEnableState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetWifiEnableState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETWIFIENABLESTATE")
  (make-package "KORTEX_DRIVER::GETWIFIENABLESTATE"))
(unless (find-package "KORTEX_DRIVER::GETWIFIENABLESTATEREQUEST")
  (make-package "KORTEX_DRIVER::GETWIFIENABLESTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETWIFIENABLESTATERESPONSE")
  (make-package "KORTEX_DRIVER::GETWIFIENABLESTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetWifiEnableStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetWifiEnableStateRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetWifiEnableStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetWifiEnableStateResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::WifiEnableState :init))
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
    ;; kortex_driver/WifiEnableState _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiEnableState _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiEnableState _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetWifiEnableState
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetWifiEnableState :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetWifiEnableState :datatype-) "kortex_driver/GetWifiEnableState")
(setf (get kortex_driver::GetWifiEnableState :request) kortex_driver::GetWifiEnableStateRequest)
(setf (get kortex_driver::GetWifiEnableState :response) kortex_driver::GetWifiEnableStateResponse)

(defmethod kortex_driver::GetWifiEnableStateRequest
  (:response () (instance kortex_driver::GetWifiEnableStateResponse :init)))

(setf (get kortex_driver::GetWifiEnableStateRequest :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetWifiEnableStateRequest :datatype-) "kortex_driver/GetWifiEnableStateRequest")
(setf (get kortex_driver::GetWifiEnableStateRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiEnableState output

================================================================================
MSG: kortex_driver/WifiEnableState

")

(setf (get kortex_driver::GetWifiEnableStateResponse :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetWifiEnableStateResponse :datatype-) "kortex_driver/GetWifiEnableStateResponse")
(setf (get kortex_driver::GetWifiEnableStateResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WifiEnableState output

================================================================================
MSG: kortex_driver/WifiEnableState

")



(provide :kortex_driver/GetWifiEnableState "9747040002a13b23ba7503e4b2f380fb")


