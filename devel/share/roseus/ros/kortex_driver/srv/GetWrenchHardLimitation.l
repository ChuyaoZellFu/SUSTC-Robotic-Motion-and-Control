;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetWrenchHardLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetWrenchHardLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETWRENCHHARDLIMITATION")
  (make-package "KORTEX_DRIVER::GETWRENCHHARDLIMITATION"))
(unless (find-package "KORTEX_DRIVER::GETWRENCHHARDLIMITATIONREQUEST")
  (make-package "KORTEX_DRIVER::GETWRENCHHARDLIMITATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETWRENCHHARDLIMITATIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETWRENCHHARDLIMITATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetWrenchHardLimitationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetWrenchHardLimitationRequest
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

(defclass kortex_driver::GetWrenchHardLimitationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetWrenchHardLimitationResponse
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

(defclass kortex_driver::GetWrenchHardLimitation
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetWrenchHardLimitation :md5sum-) "d1a67d52edbd3c6e07b5922a22648cf3")
(setf (get kortex_driver::GetWrenchHardLimitation :datatype-) "kortex_driver/GetWrenchHardLimitation")
(setf (get kortex_driver::GetWrenchHardLimitation :request) kortex_driver::GetWrenchHardLimitationRequest)
(setf (get kortex_driver::GetWrenchHardLimitation :response) kortex_driver::GetWrenchHardLimitationResponse)

(defmethod kortex_driver::GetWrenchHardLimitationRequest
  (:response () (instance kortex_driver::GetWrenchHardLimitationResponse :init)))

(setf (get kortex_driver::GetWrenchHardLimitationRequest :md5sum-) "d1a67d52edbd3c6e07b5922a22648cf3")
(setf (get kortex_driver::GetWrenchHardLimitationRequest :datatype-) "kortex_driver/GetWrenchHardLimitationRequest")
(setf (get kortex_driver::GetWrenchHardLimitationRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WrenchLimitation output

================================================================================
MSG: kortex_driver/WrenchLimitation

float32 force
")

(setf (get kortex_driver::GetWrenchHardLimitationResponse :md5sum-) "d1a67d52edbd3c6e07b5922a22648cf3")
(setf (get kortex_driver::GetWrenchHardLimitationResponse :datatype-) "kortex_driver/GetWrenchHardLimitationResponse")
(setf (get kortex_driver::GetWrenchHardLimitationResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
WrenchLimitation output

================================================================================
MSG: kortex_driver/WrenchLimitation

float32 force
")



(provide :kortex_driver/GetWrenchHardLimitation "d1a67d52edbd3c6e07b5922a22648cf3")


