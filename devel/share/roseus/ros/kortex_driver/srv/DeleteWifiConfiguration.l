;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteWifiConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteWifiConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETEWIFICONFIGURATION")
  (make-package "KORTEX_DRIVER::DELETEWIFICONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::DELETEWIFICONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::DELETEWIFICONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETEWIFICONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::DELETEWIFICONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteWifiConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteWifiConfigurationRequest
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

(defclass kortex_driver::DeleteWifiConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteWifiConfigurationResponse
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

(defclass kortex_driver::DeleteWifiConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteWifiConfiguration :md5sum-) "fc936133533cfced4ae662bec0d72a39")
(setf (get kortex_driver::DeleteWifiConfiguration :datatype-) "kortex_driver/DeleteWifiConfiguration")
(setf (get kortex_driver::DeleteWifiConfiguration :request) kortex_driver::DeleteWifiConfigurationRequest)
(setf (get kortex_driver::DeleteWifiConfiguration :response) kortex_driver::DeleteWifiConfigurationResponse)

(defmethod kortex_driver::DeleteWifiConfigurationRequest
  (:response () (instance kortex_driver::DeleteWifiConfigurationResponse :init)))

(setf (get kortex_driver::DeleteWifiConfigurationRequest :md5sum-) "fc936133533cfced4ae662bec0d72a39")
(setf (get kortex_driver::DeleteWifiConfigurationRequest :datatype-) "kortex_driver/DeleteWifiConfigurationRequest")
(setf (get kortex_driver::DeleteWifiConfigurationRequest :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteWifiConfigurationResponse :md5sum-) "fc936133533cfced4ae662bec0d72a39")
(setf (get kortex_driver::DeleteWifiConfigurationResponse :datatype-) "kortex_driver/DeleteWifiConfigurationResponse")
(setf (get kortex_driver::DeleteWifiConfigurationResponse :definition-)
      "Ssid input

================================================================================
MSG: kortex_driver/Ssid

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteWifiConfiguration "fc936133533cfced4ae662bec0d72a39")


