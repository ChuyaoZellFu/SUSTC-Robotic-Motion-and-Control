;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetPartNumber)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetPartNumber (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETPARTNUMBER")
  (make-package "KORTEX_DRIVER::GETPARTNUMBER"))
(unless (find-package "KORTEX_DRIVER::GETPARTNUMBERREQUEST")
  (make-package "KORTEX_DRIVER::GETPARTNUMBERREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETPARTNUMBERRESPONSE")
  (make-package "KORTEX_DRIVER::GETPARTNUMBERRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetPartNumberRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetPartNumberRequest
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

(defclass kortex_driver::GetPartNumberResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetPartNumberResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::PartNumber :init))
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
    ;; kortex_driver/PartNumber _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/PartNumber _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/PartNumber _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetPartNumber
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetPartNumber :md5sum-) "5989c9475bfe9b2d805af628c388fd87")
(setf (get kortex_driver::GetPartNumber :datatype-) "kortex_driver/GetPartNumber")
(setf (get kortex_driver::GetPartNumber :request) kortex_driver::GetPartNumberRequest)
(setf (get kortex_driver::GetPartNumber :response) kortex_driver::GetPartNumberResponse)

(defmethod kortex_driver::GetPartNumberRequest
  (:response () (instance kortex_driver::GetPartNumberResponse :init)))

(setf (get kortex_driver::GetPartNumberRequest :md5sum-) "5989c9475bfe9b2d805af628c388fd87")
(setf (get kortex_driver::GetPartNumberRequest :datatype-) "kortex_driver/GetPartNumberRequest")
(setf (get kortex_driver::GetPartNumberRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
PartNumber output

================================================================================
MSG: kortex_driver/PartNumber

")

(setf (get kortex_driver::GetPartNumberResponse :md5sum-) "5989c9475bfe9b2d805af628c388fd87")
(setf (get kortex_driver::GetPartNumberResponse :datatype-) "kortex_driver/GetPartNumberResponse")
(setf (get kortex_driver::GetPartNumberResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
PartNumber output

================================================================================
MSG: kortex_driver/PartNumber

")



(provide :kortex_driver/GetPartNumber "5989c9475bfe9b2d805af628c388fd87")


