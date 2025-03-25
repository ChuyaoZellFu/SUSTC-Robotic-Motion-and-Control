;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetOptionInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetOptionInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETOPTIONINFORMATION")
  (make-package "KORTEX_DRIVER::GETOPTIONINFORMATION"))
(unless (find-package "KORTEX_DRIVER::GETOPTIONINFORMATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETOPTIONINFORMATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETOPTIONINFORMATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETOPTIONINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetOptionInformationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetOptionInformationRequest
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

(defclass kortex_driver::GetOptionInformationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetOptionInformationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::OptionInformation :init))
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
    ;; kortex_driver/OptionInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/OptionInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/OptionInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetOptionInformation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetOptionInformation :md5sum-) "0af2e58d9822a8c26f4c5a66d0d6015b")
(setf (get kortex_driver::GetOptionInformation :datatype-) "kortex_driver/GetOptionInformation")
(setf (get kortex_driver::GetOptionInformation :request) kortex_driver::GetOptionInformationRequest)
(setf (get kortex_driver::GetOptionInformation :response) kortex_driver::GetOptionInformationResponse)

(defmethod kortex_driver::GetOptionInformationRequest
  (:response () (instance kortex_driver::GetOptionInformationResponse :init)))

(setf (get kortex_driver::GetOptionInformationRequest :md5sum-) "0af2e58d9822a8c26f4c5a66d0d6015b")
(setf (get kortex_driver::GetOptionInformationRequest :datatype-) "kortex_driver/GetOptionInformationRequest")
(setf (get kortex_driver::GetOptionInformationRequest :definition-)
      "OptionIdentifier input

================================================================================
MSG: kortex_driver/OptionIdentifier

uint32 sensor
---
OptionInformation output

================================================================================
MSG: kortex_driver/OptionInformation

uint32 sensor
uint32 option
bool supported
bool read_only
float32 minimum
float32 maximum
float32 step
")

(setf (get kortex_driver::GetOptionInformationResponse :md5sum-) "0af2e58d9822a8c26f4c5a66d0d6015b")
(setf (get kortex_driver::GetOptionInformationResponse :datatype-) "kortex_driver/GetOptionInformationResponse")
(setf (get kortex_driver::GetOptionInformationResponse :definition-)
      "OptionIdentifier input

================================================================================
MSG: kortex_driver/OptionIdentifier

uint32 sensor
---
OptionInformation output

================================================================================
MSG: kortex_driver/OptionInformation

uint32 sensor
uint32 option
bool supported
bool read_only
float32 minimum
float32 maximum
float32 step
")



(provide :kortex_driver/GetOptionInformation "0af2e58d9822a8c26f4c5a66d0d6015b")


