;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PauseSequence)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PauseSequence (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PAUSESEQUENCE")
  (make-package "KORTEX_DRIVER::PAUSESEQUENCE"))
(unless (find-package "KORTEX_DRIVER::PAUSESEQUENCEREQUEST")
  (make-package "KORTEX_DRIVER::PAUSESEQUENCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::PAUSESEQUENCERESPONSE")
  (make-package "KORTEX_DRIVER::PAUSESEQUENCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PauseSequenceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PauseSequenceRequest
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

(defclass kortex_driver::PauseSequenceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PauseSequenceResponse
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

(defclass kortex_driver::PauseSequence
  :super ros::object
  :slots ())

(setf (get kortex_driver::PauseSequence :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::PauseSequence :datatype-) "kortex_driver/PauseSequence")
(setf (get kortex_driver::PauseSequence :request) kortex_driver::PauseSequenceRequest)
(setf (get kortex_driver::PauseSequence :response) kortex_driver::PauseSequenceResponse)

(defmethod kortex_driver::PauseSequenceRequest
  (:response () (instance kortex_driver::PauseSequenceResponse :init)))

(setf (get kortex_driver::PauseSequenceRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::PauseSequenceRequest :datatype-) "kortex_driver/PauseSequenceRequest")
(setf (get kortex_driver::PauseSequenceRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PauseSequenceResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::PauseSequenceResponse :datatype-) "kortex_driver/PauseSequenceResponse")
(setf (get kortex_driver::PauseSequenceResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PauseSequence "f335b819dc59099fe3124b36f140ad07")


