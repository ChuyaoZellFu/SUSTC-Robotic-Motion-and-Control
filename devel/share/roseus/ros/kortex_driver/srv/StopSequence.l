;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StopSequence)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StopSequence (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STOPSEQUENCE")
  (make-package "KORTEX_DRIVER::STOPSEQUENCE"))
(unless (find-package "KORTEX_DRIVER::STOPSEQUENCEREQUEST")
  (make-package "KORTEX_DRIVER::STOPSEQUENCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::STOPSEQUENCERESPONSE")
  (make-package "KORTEX_DRIVER::STOPSEQUENCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::StopSequenceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::StopSequenceRequest
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

(defclass kortex_driver::StopSequenceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::StopSequenceResponse
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

(defclass kortex_driver::StopSequence
  :super ros::object
  :slots ())

(setf (get kortex_driver::StopSequence :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopSequence :datatype-) "kortex_driver/StopSequence")
(setf (get kortex_driver::StopSequence :request) kortex_driver::StopSequenceRequest)
(setf (get kortex_driver::StopSequence :response) kortex_driver::StopSequenceResponse)

(defmethod kortex_driver::StopSequenceRequest
  (:response () (instance kortex_driver::StopSequenceResponse :init)))

(setf (get kortex_driver::StopSequenceRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopSequenceRequest :datatype-) "kortex_driver/StopSequenceRequest")
(setf (get kortex_driver::StopSequenceRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::StopSequenceResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopSequenceResponse :datatype-) "kortex_driver/StopSequenceResponse")
(setf (get kortex_driver::StopSequenceResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/StopSequence "f335b819dc59099fe3124b36f140ad07")


