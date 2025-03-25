;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetIPv4Settings)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetIPv4Settings (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETIPV4SETTINGS")
  (make-package "KORTEX_DRIVER::SETIPV4SETTINGS"))
(unless (find-package "KORTEX_DRIVER::SETIPV4SETTINGSREQUEST")
  (make-package "KORTEX_DRIVER::SETIPV4SETTINGSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETIPV4SETTINGSRESPONSE")
  (make-package "KORTEX_DRIVER::SETIPV4SETTINGSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetIPv4SettingsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetIPv4SettingsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::IPv4Settings :init))
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
    ;; kortex_driver/IPv4Settings _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/IPv4Settings _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/IPv4Settings _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetIPv4SettingsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetIPv4SettingsResponse
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

(defclass kortex_driver::SetIPv4Settings
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetIPv4Settings :md5sum-) "361b42ee5a195ec231ecbbedd2098401")
(setf (get kortex_driver::SetIPv4Settings :datatype-) "kortex_driver/SetIPv4Settings")
(setf (get kortex_driver::SetIPv4Settings :request) kortex_driver::SetIPv4SettingsRequest)
(setf (get kortex_driver::SetIPv4Settings :response) kortex_driver::SetIPv4SettingsResponse)

(defmethod kortex_driver::SetIPv4SettingsRequest
  (:response () (instance kortex_driver::SetIPv4SettingsResponse :init)))

(setf (get kortex_driver::SetIPv4SettingsRequest :md5sum-) "361b42ee5a195ec231ecbbedd2098401")
(setf (get kortex_driver::SetIPv4SettingsRequest :datatype-) "kortex_driver/SetIPv4SettingsRequest")
(setf (get kortex_driver::SetIPv4SettingsRequest :definition-)
      "IPv4Settings input

================================================================================
MSG: kortex_driver/IPv4Settings

uint32 ipv4_address
uint32 ipv4_subnet_mask
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetIPv4SettingsResponse :md5sum-) "361b42ee5a195ec231ecbbedd2098401")
(setf (get kortex_driver::SetIPv4SettingsResponse :datatype-) "kortex_driver/SetIPv4SettingsResponse")
(setf (get kortex_driver::SetIPv4SettingsResponse :definition-)
      "IPv4Settings input

================================================================================
MSG: kortex_driver/IPv4Settings

uint32 ipv4_address
uint32 ipv4_subnet_mask
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetIPv4Settings "361b42ee5a195ec231ecbbedd2098401")


