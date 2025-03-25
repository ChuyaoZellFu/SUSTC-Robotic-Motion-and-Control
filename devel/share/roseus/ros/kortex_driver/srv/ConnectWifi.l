;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ConnectWifi)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ConnectWifi (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONNECTWIFI")
  (make-package "KORTEX_DRIVER::CONNECTWIFI"))
(unless (find-package "KORTEX_DRIVER::CONNECTWIFIREQUEST")
  (make-package "KORTEX_DRIVER::CONNECTWIFIREQUEST"))
(unless (find-package "KORTEX_DRIVER::CONNECTWIFIRESPONSE")
  (make-package "KORTEX_DRIVER::CONNECTWIFIRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ConnectWifiRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ConnectWifiRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Ssid :init))
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
    ;; kortex_driver/Ssid _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Ssid _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Ssid _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ConnectWifiResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ConnectWifiResponse
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

(defclass kortex_driver::ConnectWifi
  :super ros::object
  :slots ())

(setf (get kortex_driver::ConnectWifi :md5sum-) "fc936133533cfced4ae662bec0d72a39")
(setf (get kortex_driver::ConnectWifi :datatype-) "kortex_driver/ConnectWifi")
(setf (get kortex_driver::ConnectWifi :request) kortex_driver::ConnectWifiRequest)
(setf (get kortex_driver::ConnectWifi :response) kortex_driver::ConnectWifiResponse)

(defmethod kortex_driver::ConnectWifiRequest
  (:response () (instance kortex_driver::ConnectWifiResponse :init)))

(setf (get kortex_driver::ConnectWifiRequest :md5sum-) "fc936133533cfced4ae662bec0d72a39")
(setf (get kortex_driver::ConnectWifiRequest :datatype-) "kortex_driver/ConnectWifiRequest")
(setf (get kortex_driver::ConnectWifiRequest :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ConnectWifiResponse :md5sum-) "fc936133533cfced4ae662bec0d72a39")
(setf (get kortex_driver::ConnectWifiResponse :datatype-) "kortex_driver/ConnectWifiResponse")
(setf (get kortex_driver::ConnectWifiResponse :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ConnectWifi "fc936133533cfced4ae662bec0d72a39")


