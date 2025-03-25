;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetOptionValue)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetOptionValue (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETOPTIONVALUE")
  (make-package "KORTEX_DRIVER::GETOPTIONVALUE"))
(unless (find-package "KORTEX_DRIVER::GETOPTIONVALUEREQUEST")
  (make-package "KORTEX_DRIVER::GETOPTIONVALUEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETOPTIONVALUERESPONSE")
  (make-package "KORTEX_DRIVER::GETOPTIONVALUERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetOptionValueRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetOptionValueRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::OptionIdentifier :init))
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
    ;; kortex_driver/OptionIdentifier _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/OptionIdentifier _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/OptionIdentifier _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetOptionValueResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetOptionValueResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::OptionValue :init))
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
    ;; kortex_driver/OptionValue _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/OptionValue _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/OptionValue _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetOptionValue
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetOptionValue :md5sum-) "e414d686b57953b5748ab5bedaec5577")
(setf (get kortex_driver::GetOptionValue :datatype-) "kortex_driver/GetOptionValue")
(setf (get kortex_driver::GetOptionValue :request) kortex_driver::GetOptionValueRequest)
(setf (get kortex_driver::GetOptionValue :response) kortex_driver::GetOptionValueResponse)

(defmethod kortex_driver::GetOptionValueRequest
  (:response () (instance kortex_driver::GetOptionValueResponse :init)))

(setf (get kortex_driver::GetOptionValueRequest :md5sum-) "e414d686b57953b5748ab5bedaec5577")
(setf (get kortex_driver::GetOptionValueRequest :datatype-) "kortex_driver/GetOptionValueRequest")
(setf (get kortex_driver::GetOptionValueRequest :definition-)
      "OptionIdentifier input

================================================================================
MSG: kortex_driver/OptionIdentifier

uint32 sensor
---
OptionValue output

================================================================================
MSG: kortex_driver/OptionValue

uint32 sensor
uint32 option
")

(setf (get kortex_driver::GetOptionValueResponse :md5sum-) "e414d686b57953b5748ab5bedaec5577")
(setf (get kortex_driver::GetOptionValueResponse :datatype-) "kortex_driver/GetOptionValueResponse")
(setf (get kortex_driver::GetOptionValueResponse :definition-)
      "OptionIdentifier input

================================================================================
MSG: kortex_driver/OptionIdentifier

uint32 sensor
---
OptionValue output

================================================================================
MSG: kortex_driver/OptionValue

uint32 sensor
uint32 option
")



(provide :kortex_driver/GetOptionValue "e414d686b57953b5748ab5bedaec5577")


