;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetEthernetConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetEthernetConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETETHERNETCONFIGURATION")
  (make-package "KORTEX_DRIVER::SETETHERNETCONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::SETETHERNETCONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::SETETHERNETCONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETETHERNETCONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::SETETHERNETCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetEthernetConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetEthernetConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::EthernetConfiguration :init))
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
    ;; kortex_driver/EthernetConfiguration _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/EthernetConfiguration _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/EthernetConfiguration _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetEthernetConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetEthernetConfigurationResponse
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

(defclass kortex_driver::SetEthernetConfiguration
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetEthernetConfiguration :md5sum-) "fdfcb3d5cd015a2a65d4aee878ff80be")
(setf (get kortex_driver::SetEthernetConfiguration :datatype-) "kortex_driver/SetEthernetConfiguration")
(setf (get kortex_driver::SetEthernetConfiguration :request) kortex_driver::SetEthernetConfigurationRequest)
(setf (get kortex_driver::SetEthernetConfiguration :response) kortex_driver::SetEthernetConfigurationResponse)

(defmethod kortex_driver::SetEthernetConfigurationRequest
  (:response () (instance kortex_driver::SetEthernetConfigurationResponse :init)))

(setf (get kortex_driver::SetEthernetConfigurationRequest :md5sum-) "fdfcb3d5cd015a2a65d4aee878ff80be")
(setf (get kortex_driver::SetEthernetConfigurationRequest :datatype-) "kortex_driver/SetEthernetConfigurationRequest")
(setf (get kortex_driver::SetEthernetConfigurationRequest :definition-)
      "EthernetConfiguration input

================================================================================
MSG: kortex_driver/EthernetConfiguration

uint32 device
bool enabled
uint32 speed
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetEthernetConfigurationResponse :md5sum-) "fdfcb3d5cd015a2a65d4aee878ff80be")
(setf (get kortex_driver::SetEthernetConfigurationResponse :datatype-) "kortex_driver/SetEthernetConfigurationResponse")
(setf (get kortex_driver::SetEthernetConfigurationResponse :definition-)
      "EthernetConfiguration input

================================================================================
MSG: kortex_driver/EthernetConfiguration

uint32 device
bool enabled
uint32 speed
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetEthernetConfiguration "fdfcb3d5cd015a2a65d4aee878ff80be")


