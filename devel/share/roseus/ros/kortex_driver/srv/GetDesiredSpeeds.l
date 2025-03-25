;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetDesiredSpeeds)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetDesiredSpeeds (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETDESIREDSPEEDS")
  (make-package "KORTEX_DRIVER::GETDESIREDSPEEDS"))
(unless (find-package "KORTEX_DRIVER::GETDESIREDSPEEDSREQUEST")
  (make-package "KORTEX_DRIVER::GETDESIREDSPEEDSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETDESIREDSPEEDSRESPONSE")
  (make-package "KORTEX_DRIVER::GETDESIREDSPEEDSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetDesiredSpeedsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetDesiredSpeedsRequest
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

(defclass kortex_driver::GetDesiredSpeedsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetDesiredSpeedsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::DesiredSpeeds :init))
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
    ;; kortex_driver/DesiredSpeeds _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/DesiredSpeeds _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/DesiredSpeeds _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetDesiredSpeeds
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetDesiredSpeeds :md5sum-) "1dd665a1d839a65c717e02f2317ca84c")
(setf (get kortex_driver::GetDesiredSpeeds :datatype-) "kortex_driver/GetDesiredSpeeds")
(setf (get kortex_driver::GetDesiredSpeeds :request) kortex_driver::GetDesiredSpeedsRequest)
(setf (get kortex_driver::GetDesiredSpeeds :response) kortex_driver::GetDesiredSpeedsResponse)

(defmethod kortex_driver::GetDesiredSpeedsRequest
  (:response () (instance kortex_driver::GetDesiredSpeedsResponse :init)))

(setf (get kortex_driver::GetDesiredSpeedsRequest :md5sum-) "1dd665a1d839a65c717e02f2317ca84c")
(setf (get kortex_driver::GetDesiredSpeedsRequest :datatype-) "kortex_driver/GetDesiredSpeedsRequest")
(setf (get kortex_driver::GetDesiredSpeedsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
DesiredSpeeds output

================================================================================
MSG: kortex_driver/DesiredSpeeds

float32 linear
float32 angular
")

(setf (get kortex_driver::GetDesiredSpeedsResponse :md5sum-) "1dd665a1d839a65c717e02f2317ca84c")
(setf (get kortex_driver::GetDesiredSpeedsResponse :datatype-) "kortex_driver/GetDesiredSpeedsResponse")
(setf (get kortex_driver::GetDesiredSpeedsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
DesiredSpeeds output

================================================================================
MSG: kortex_driver/DesiredSpeeds

float32 linear
float32 angular
")



(provide :kortex_driver/GetDesiredSpeeds "1dd665a1d839a65c717e02f2317ca84c")


