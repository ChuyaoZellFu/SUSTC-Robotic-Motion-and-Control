;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetSerialNumber)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetSerialNumber (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSERIALNUMBER")
  (make-package "KORTEX_DRIVER::GETSERIALNUMBER"))
(unless (find-package "KORTEX_DRIVER::GETSERIALNUMBERREQUEST")
  (make-package "KORTEX_DRIVER::GETSERIALNUMBERREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSERIALNUMBERRESPONSE")
  (make-package "KORTEX_DRIVER::GETSERIALNUMBERRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetSerialNumberRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetSerialNumberRequest
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

(defclass kortex_driver::GetSerialNumberResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetSerialNumberResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::SerialNumber :init))
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
    ;; kortex_driver/SerialNumber _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SerialNumber _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SerialNumber _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSerialNumber
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetSerialNumber :md5sum-) "0249587230c33b79820737b46150559e")
(setf (get kortex_driver::GetSerialNumber :datatype-) "kortex_driver/GetSerialNumber")
(setf (get kortex_driver::GetSerialNumber :request) kortex_driver::GetSerialNumberRequest)
(setf (get kortex_driver::GetSerialNumber :response) kortex_driver::GetSerialNumberResponse)

(defmethod kortex_driver::GetSerialNumberRequest
  (:response () (instance kortex_driver::GetSerialNumberResponse :init)))

(setf (get kortex_driver::GetSerialNumberRequest :md5sum-) "0249587230c33b79820737b46150559e")
(setf (get kortex_driver::GetSerialNumberRequest :datatype-) "kortex_driver/GetSerialNumberRequest")
(setf (get kortex_driver::GetSerialNumberRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
SerialNumber output

================================================================================
MSG: kortex_driver/SerialNumber

")

(setf (get kortex_driver::GetSerialNumberResponse :md5sum-) "0249587230c33b79820737b46150559e")
(setf (get kortex_driver::GetSerialNumberResponse :datatype-) "kortex_driver/GetSerialNumberResponse")
(setf (get kortex_driver::GetSerialNumberResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
SerialNumber output

================================================================================
MSG: kortex_driver/SerialNumber

")



(provide :kortex_driver/GetSerialNumber "0249587230c33b79820737b46150559e")


