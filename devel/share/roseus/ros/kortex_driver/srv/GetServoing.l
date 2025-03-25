;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetServoing)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetServoing (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSERVOING")
  (make-package "KORTEX_DRIVER::GETSERVOING"))
(unless (find-package "KORTEX_DRIVER::GETSERVOINGREQUEST")
  (make-package "KORTEX_DRIVER::GETSERVOINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSERVOINGRESPONSE")
  (make-package "KORTEX_DRIVER::GETSERVOINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetServoingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetServoingRequest
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

(defclass kortex_driver::GetServoingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetServoingResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Servoing :init))
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
    ;; kortex_driver/Servoing _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Servoing _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Servoing _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetServoing
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetServoing :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetServoing :datatype-) "kortex_driver/GetServoing")
(setf (get kortex_driver::GetServoing :request) kortex_driver::GetServoingRequest)
(setf (get kortex_driver::GetServoing :response) kortex_driver::GetServoingResponse)

(defmethod kortex_driver::GetServoingRequest
  (:response () (instance kortex_driver::GetServoingResponse :init)))

(setf (get kortex_driver::GetServoingRequest :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetServoingRequest :datatype-) "kortex_driver/GetServoingRequest")
(setf (get kortex_driver::GetServoingRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Servoing output

================================================================================
MSG: kortex_driver/Servoing

")

(setf (get kortex_driver::GetServoingResponse :md5sum-) "9747040002a13b23ba7503e4b2f380fb")
(setf (get kortex_driver::GetServoingResponse :datatype-) "kortex_driver/GetServoingResponse")
(setf (get kortex_driver::GetServoingResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Servoing output

================================================================================
MSG: kortex_driver/Servoing

")



(provide :kortex_driver/GetServoing "9747040002a13b23ba7503e4b2f380fb")


