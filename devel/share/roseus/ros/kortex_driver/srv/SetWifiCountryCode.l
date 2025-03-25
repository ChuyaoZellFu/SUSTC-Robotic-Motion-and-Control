;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetWifiCountryCode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetWifiCountryCode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETWIFICOUNTRYCODE")
  (make-package "KORTEX_DRIVER::SETWIFICOUNTRYCODE"))
(unless (find-package "KORTEX_DRIVER::SETWIFICOUNTRYCODEREQUEST")
  (make-package "KORTEX_DRIVER::SETWIFICOUNTRYCODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETWIFICOUNTRYCODERESPONSE")
  (make-package "KORTEX_DRIVER::SETWIFICOUNTRYCODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetWifiCountryCodeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetWifiCountryCodeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::CountryCode :init))
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
    ;; kortex_driver/CountryCode _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CountryCode _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CountryCode _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetWifiCountryCodeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetWifiCountryCodeResponse
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

(defclass kortex_driver::SetWifiCountryCode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetWifiCountryCode :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::SetWifiCountryCode :datatype-) "kortex_driver/SetWifiCountryCode")
(setf (get kortex_driver::SetWifiCountryCode :request) kortex_driver::SetWifiCountryCodeRequest)
(setf (get kortex_driver::SetWifiCountryCode :response) kortex_driver::SetWifiCountryCodeResponse)

(defmethod kortex_driver::SetWifiCountryCodeRequest
  (:response () (instance kortex_driver::SetWifiCountryCodeResponse :init)))

(setf (get kortex_driver::SetWifiCountryCodeRequest :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::SetWifiCountryCodeRequest :datatype-) "kortex_driver/SetWifiCountryCodeRequest")
(setf (get kortex_driver::SetWifiCountryCodeRequest :definition-)
      "CountryCode input

================================================================================
MSG: kortex_driver/CountryCode

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetWifiCountryCodeResponse :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::SetWifiCountryCodeResponse :datatype-) "kortex_driver/SetWifiCountryCodeResponse")
(setf (get kortex_driver::SetWifiCountryCodeResponse :definition-)
      "CountryCode input

================================================================================
MSG: kortex_driver/CountryCode

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetWifiCountryCode "7960dab80443332660b3bb4bf774c9ab")


