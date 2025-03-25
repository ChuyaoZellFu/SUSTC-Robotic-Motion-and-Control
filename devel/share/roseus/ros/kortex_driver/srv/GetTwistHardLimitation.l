;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetTwistHardLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetTwistHardLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETTWISTHARDLIMITATION")
  (make-package "KORTEX_DRIVER::GETTWISTHARDLIMITATION"))
(unless (find-package "KORTEX_DRIVER::GETTWISTHARDLIMITATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETTWISTHARDLIMITATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETTWISTHARDLIMITATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETTWISTHARDLIMITATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetTwistHardLimitationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetTwistHardLimitationRequest
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

(defclass kortex_driver::GetTwistHardLimitationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetTwistHardLimitationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::TwistLimitation :init))
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
    ;; kortex_driver/TwistLimitation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TwistLimitation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TwistLimitation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetTwistHardLimitation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetTwistHardLimitation :md5sum-) "69b1f43d97a46e21f8b8586022fd23ec")
(setf (get kortex_driver::GetTwistHardLimitation :datatype-) "kortex_driver/GetTwistHardLimitation")
(setf (get kortex_driver::GetTwistHardLimitation :request) kortex_driver::GetTwistHardLimitationRequest)
(setf (get kortex_driver::GetTwistHardLimitation :response) kortex_driver::GetTwistHardLimitationResponse)

(defmethod kortex_driver::GetTwistHardLimitationRequest
  (:response () (instance kortex_driver::GetTwistHardLimitationResponse :init)))

(setf (get kortex_driver::GetTwistHardLimitationRequest :md5sum-) "69b1f43d97a46e21f8b8586022fd23ec")
(setf (get kortex_driver::GetTwistHardLimitationRequest :datatype-) "kortex_driver/GetTwistHardLimitationRequest")
(setf (get kortex_driver::GetTwistHardLimitationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
TwistLimitation output

================================================================================
MSG: kortex_driver/TwistLimitation

float32 linear
")

(setf (get kortex_driver::GetTwistHardLimitationResponse :md5sum-) "69b1f43d97a46e21f8b8586022fd23ec")
(setf (get kortex_driver::GetTwistHardLimitationResponse :datatype-) "kortex_driver/GetTwistHardLimitationResponse")
(setf (get kortex_driver::GetTwistHardLimitationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
TwistLimitation output

================================================================================
MSG: kortex_driver/TwistLimitation

float32 linear
")



(provide :kortex_driver/GetTwistHardLimitation "69b1f43d97a46e21f8b8586022fd23ec")


