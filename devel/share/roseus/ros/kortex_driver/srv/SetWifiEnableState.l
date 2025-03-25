;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetWifiEnableState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetWifiEnableState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETWIFIENABLESTATE")
  (make-package "KORTEX_DRIVER::SETWIFIENABLESTATE"))
(unless (find-package "KORTEX_DRIVER::SETWIFIENABLESTATEREQUEST")
  (make-package "KORTEX_DRIVER::SETWIFIENABLESTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETWIFIENABLESTATERESPONSE")
  (make-package "KORTEX_DRIVER::SETWIFIENABLESTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetWifiEnableStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetWifiEnableStateRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::WifiEnableState :init))
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
    ;; kortex_driver/WifiEnableState _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WifiEnableState _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WifiEnableState _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetWifiEnableStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetWifiEnableStateResponse
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

(defclass kortex_driver::SetWifiEnableState
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetWifiEnableState :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetWifiEnableState :datatype-) "kortex_driver/SetWifiEnableState")
(setf (get kortex_driver::SetWifiEnableState :request) kortex_driver::SetWifiEnableStateRequest)
(setf (get kortex_driver::SetWifiEnableState :response) kortex_driver::SetWifiEnableStateResponse)

(defmethod kortex_driver::SetWifiEnableStateRequest
  (:response () (instance kortex_driver::SetWifiEnableStateResponse :init)))

(setf (get kortex_driver::SetWifiEnableStateRequest :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetWifiEnableStateRequest :datatype-) "kortex_driver/SetWifiEnableStateRequest")
(setf (get kortex_driver::SetWifiEnableStateRequest :definition-)
      "WifiEnableState input

================================================================================
MSG: kortex_driver/WifiEnableState

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetWifiEnableStateResponse :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetWifiEnableStateResponse :datatype-) "kortex_driver/SetWifiEnableStateResponse")
(setf (get kortex_driver::SetWifiEnableStateResponse :definition-)
      "WifiEnableState input

================================================================================
MSG: kortex_driver/WifiEnableState

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetWifiEnableState "7ccd6a6c56b1642433541444d0e2ab2f")


