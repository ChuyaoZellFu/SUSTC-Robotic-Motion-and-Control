;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetIPv4Information)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetIPv4Information (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETIPV4INFORMATION")
  (make-package "KORTEX_DRIVER::GETIPV4INFORMATION"))
(unless (find-package "KORTEX_DRIVER::GETIPV4INFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETIPV4INFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETIPV4INFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETIPV4INFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetIPv4InformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetIPv4InformationRequest
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

(defclass kortex_driver::GetIPv4InformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetIPv4InformationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::IPv4Information :init))
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
    ;; kortex_driver/IPv4Information _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/IPv4Information _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/IPv4Information _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetIPv4Information
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetIPv4Information :md5sum-) "cf6aba6a62d331d4b104d99fe292f36c")
(setf (get kortex_driver::GetIPv4Information :datatype-) "kortex_driver/GetIPv4Information")
(setf (get kortex_driver::GetIPv4Information :request) kortex_driver::GetIPv4InformationRequest)
(setf (get kortex_driver::GetIPv4Information :response) kortex_driver::GetIPv4InformationResponse)

(defmethod kortex_driver::GetIPv4InformationRequest
  (:response () (instance kortex_driver::GetIPv4InformationResponse :init)))

(setf (get kortex_driver::GetIPv4InformationRequest :md5sum-) "cf6aba6a62d331d4b104d99fe292f36c")
(setf (get kortex_driver::GetIPv4InformationRequest :datatype-) "kortex_driver/GetIPv4InformationRequest")
(setf (get kortex_driver::GetIPv4InformationRequest :definition-)
      "NetworkHandle input

================================================================================
MSG: kortex_driver/NetworkHandle

---
IPv4Information output

================================================================================
MSG: kortex_driver/IPv4Information

uint32 ip_address
uint32 subnet_mask
")

(setf (get kortex_driver::GetIPv4InformationResponse :md5sum-) "cf6aba6a62d331d4b104d99fe292f36c")
(setf (get kortex_driver::GetIPv4InformationResponse :datatype-) "kortex_driver/GetIPv4InformationResponse")
(setf (get kortex_driver::GetIPv4InformationResponse :definition-)
      "NetworkHandle input

================================================================================
MSG: kortex_driver/NetworkHandle

---
IPv4Information output

================================================================================
MSG: kortex_driver/IPv4Information

uint32 ip_address
uint32 subnet_mask
")



(provide :kortex_driver/GetIPv4Information "cf6aba6a62d331d4b104d99fe292f36c")


