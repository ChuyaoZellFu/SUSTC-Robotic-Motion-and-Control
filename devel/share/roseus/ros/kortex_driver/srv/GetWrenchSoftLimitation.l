;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetWrenchSoftLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetWrenchSoftLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETWRENCHSOFTLIMITATION")
  (make-package "KORTEX_DRIVER::GETWRENCHSOFTLIMITATION"))
(unless (find-package "KORTEX_DRIVER::GETWRENCHSOFTLIMITATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETWRENCHSOFTLIMITATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETWRENCHSOFTLIMITATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETWRENCHSOFTLIMITATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetWrenchSoftLimitationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetWrenchSoftLimitationRequest
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

(defclass kortex_driver::GetWrenchSoftLimitationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetWrenchSoftLimitationResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::WrenchLimitation :init))
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
    ;; kortex_driver/WrenchLimitation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/WrenchLimitation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/WrenchLimitation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetWrenchSoftLimitation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetWrenchSoftLimitation :md5sum-) "d1a67d52edbd3c6e07b5922a22648cf3")
(setf (get kortex_driver::GetWrenchSoftLimitation :datatype-) "kortex_driver/GetWrenchSoftLimitation")
(setf (get kortex_driver::GetWrenchSoftLimitation :request) kortex_driver::GetWrenchSoftLimitationRequest)
(setf (get kortex_driver::GetWrenchSoftLimitation :response) kortex_driver::GetWrenchSoftLimitationResponse)

(defmethod kortex_driver::GetWrenchSoftLimitationRequest
  (:response () (instance kortex_driver::GetWrenchSoftLimitationResponse :init)))

(setf (get kortex_driver::GetWrenchSoftLimitationRequest :md5sum-) "d1a67d52edbd3c6e07b5922a22648cf3")
(setf (get kortex_driver::GetWrenchSoftLimitationRequest :datatype-) "kortex_driver/GetWrenchSoftLimitationRequest")
(setf (get kortex_driver::GetWrenchSoftLimitationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WrenchLimitation output

================================================================================
MSG: kortex_driver/WrenchLimitation

float32 force
")

(setf (get kortex_driver::GetWrenchSoftLimitationResponse :md5sum-) "d1a67d52edbd3c6e07b5922a22648cf3")
(setf (get kortex_driver::GetWrenchSoftLimitationResponse :datatype-) "kortex_driver/GetWrenchSoftLimitationResponse")
(setf (get kortex_driver::GetWrenchSoftLimitationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WrenchLimitation output

================================================================================
MSG: kortex_driver/WrenchLimitation

float32 force
")



(provide :kortex_driver/GetWrenchSoftLimitation "d1a67d52edbd3c6e07b5922a22648cf3")


