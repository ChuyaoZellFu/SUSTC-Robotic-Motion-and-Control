;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetEthernetConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetEthernetConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETETHERNETCONFIGURATION")
  (make-package "KORTEX_DRIVER::GETETHERNETCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETETHERNETCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETETHERNETCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETETHERNETCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETETHERNETCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetEthernetConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetEthernetConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::EthernetDeviceIdentification :init))
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
    ;; kortex_driver/EthernetDeviceIdentification _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/EthernetDeviceIdentification _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/EthernetDeviceIdentification _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetEthernetConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetEthernetConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::EthernetConfiguration :init))
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
    ;; kortex_driver/EthernetConfiguration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/EthernetConfiguration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/EthernetConfiguration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetEthernetConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetEthernetConfiguration :md5sum-) "d50fb352c6ad553c74cb37db16e3d609")
(setf (get kortex_driver::GetEthernetConfiguration :datatype-) "kortex_driver/GetEthernetConfiguration")
(setf (get kortex_driver::GetEthernetConfiguration :request) kortex_driver::GetEthernetConfigurationRequest)
(setf (get kortex_driver::GetEthernetConfiguration :response) kortex_driver::GetEthernetConfigurationResponse)

(defmethod kortex_driver::GetEthernetConfigurationRequest
  (:response () (instance kortex_driver::GetEthernetConfigurationResponse :init)))

(setf (get kortex_driver::GetEthernetConfigurationRequest :md5sum-) "d50fb352c6ad553c74cb37db16e3d609")
(setf (get kortex_driver::GetEthernetConfigurationRequest :datatype-) "kortex_driver/GetEthernetConfigurationRequest")
(setf (get kortex_driver::GetEthernetConfigurationRequest :definition-)
      "EthernetDeviceIdentification input

================================================================================
MSG: kortex_driver/EthernetDeviceIdentification

---
EthernetConfiguration output

================================================================================
MSG: kortex_driver/EthernetConfiguration

uint32 device
bool enabled
uint32 speed
")

(setf (get kortex_driver::GetEthernetConfigurationResponse :md5sum-) "d50fb352c6ad553c74cb37db16e3d609")
(setf (get kortex_driver::GetEthernetConfigurationResponse :datatype-) "kortex_driver/GetEthernetConfigurationResponse")
(setf (get kortex_driver::GetEthernetConfigurationResponse :definition-)
      "EthernetDeviceIdentification input

================================================================================
MSG: kortex_driver/EthernetDeviceIdentification

---
EthernetConfiguration output

================================================================================
MSG: kortex_driver/EthernetConfiguration

uint32 device
bool enabled
uint32 speed
")



(provide :kortex_driver/GetEthernetConfiguration "d50fb352c6ad553c74cb37db16e3d609")


