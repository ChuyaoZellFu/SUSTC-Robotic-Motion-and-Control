;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetWifiCountryCode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetWifiCountryCode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETWIFICOUNTRYCODE")
  (make-package "KORTEX_DRIVER::GETWIFICOUNTRYCODE"))
(unless (find-package "KORTEX_DRIVER::GETWIFICOUNTRYCODEREQUEST")
  (make-package "KORTEX_DRIVER::GETWIFICOUNTRYCODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETWIFICOUNTRYCODERESPONSE")
  (make-package "KORTEX_DRIVER::GETWIFICOUNTRYCODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetWifiCountryCodeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetWifiCountryCodeRequest
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

(defclass kortex_driver::GetWifiCountryCodeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetWifiCountryCodeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CountryCode :init))
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
    ;; kortex_driver/CountryCode _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CountryCode _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CountryCode _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetWifiCountryCode
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetWifiCountryCode :md5sum-) "e29a445c35ddd6d1bc1a1daa766b9db1")
(setf (get kortex_driver::GetWifiCountryCode :datatype-) "kortex_driver/GetWifiCountryCode")
(setf (get kortex_driver::GetWifiCountryCode :request) kortex_driver::GetWifiCountryCodeRequest)
(setf (get kortex_driver::GetWifiCountryCode :response) kortex_driver::GetWifiCountryCodeResponse)

(defmethod kortex_driver::GetWifiCountryCodeRequest
  (:response () (instance kortex_driver::GetWifiCountryCodeResponse :init)))

(setf (get kortex_driver::GetWifiCountryCodeRequest :md5sum-) "e29a445c35ddd6d1bc1a1daa766b9db1")
(setf (get kortex_driver::GetWifiCountryCodeRequest :datatype-) "kortex_driver/GetWifiCountryCodeRequest")
(setf (get kortex_driver::GetWifiCountryCodeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CountryCode output

================================================================================
MSG: kortex_driver/CountryCode

")

(setf (get kortex_driver::GetWifiCountryCodeResponse :md5sum-) "e29a445c35ddd6d1bc1a1daa766b9db1")
(setf (get kortex_driver::GetWifiCountryCodeResponse :datatype-) "kortex_driver/GetWifiCountryCodeResponse")
(setf (get kortex_driver::GetWifiCountryCodeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CountryCode output

================================================================================
MSG: kortex_driver/CountryCode

")



(provide :kortex_driver/GetWifiCountryCode "e29a445c35ddd6d1bc1a1daa766b9db1")


