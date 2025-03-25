;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetIPv4Configuration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetIPv4Configuration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETIPV4CONFIGURATION")
  (make-package "KORTEX_DRIVER::GETIPV4CONFIGURATION"))
(unless (find-package "KORTEX_DRIVER::GETIPV4CONFIGURATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETIPV4CONFIGURATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETIPV4CONFIGURATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETIPV4CONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetIPv4ConfigurationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetIPv4ConfigurationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::NetworkHandle :init))
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
    ;; kortex_driver/NetworkHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/NetworkHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/NetworkHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetIPv4ConfigurationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetIPv4ConfigurationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::IPv4Configuration :init))
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
    ;; kortex_driver/IPv4Configuration _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/IPv4Configuration _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/IPv4Configuration _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetIPv4Configuration
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetIPv4Configuration :md5sum-) "d1ab60af0d1991268832344a0f905c68")
(setf (get kortex_driver::GetIPv4Configuration :datatype-) "kortex_driver/GetIPv4Configuration")
(setf (get kortex_driver::GetIPv4Configuration :request) kortex_driver::GetIPv4ConfigurationRequest)
(setf (get kortex_driver::GetIPv4Configuration :response) kortex_driver::GetIPv4ConfigurationResponse)

(defmethod kortex_driver::GetIPv4ConfigurationRequest
  (:response () (instance kortex_driver::GetIPv4ConfigurationResponse :init)))

(setf (get kortex_driver::GetIPv4ConfigurationRequest :md5sum-) "d1ab60af0d1991268832344a0f905c68")
(setf (get kortex_driver::GetIPv4ConfigurationRequest :datatype-) "kortex_driver/GetIPv4ConfigurationRequest")
(setf (get kortex_driver::GetIPv4ConfigurationRequest :definition-)
      "NetworkHandle input

================================================================================
MSG: kortex_driver/NetworkHandle

---
IPv4Configuration output

================================================================================
MSG: kortex_driver/IPv4Configuration

uint32 ip_address
uint32 subnet_mask
uint32 default_gateway
")

(setf (get kortex_driver::GetIPv4ConfigurationResponse :md5sum-) "d1ab60af0d1991268832344a0f905c68")
(setf (get kortex_driver::GetIPv4ConfigurationResponse :datatype-) "kortex_driver/GetIPv4ConfigurationResponse")
(setf (get kortex_driver::GetIPv4ConfigurationResponse :definition-)
      "NetworkHandle input

================================================================================
MSG: kortex_driver/NetworkHandle

---
IPv4Configuration output

================================================================================
MSG: kortex_driver/IPv4Configuration

uint32 ip_address
uint32 subnet_mask
uint32 default_gateway
")



(provide :kortex_driver/GetIPv4Configuration "d1ab60af0d1991268832344a0f905c68")


