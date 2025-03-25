;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResumeSequence)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResumeSequence (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESUMESEQUENCE")
  (make-package "KORTEX_DRIVER::RESUMESEQUENCE"))
(unless (find-package "KORTEX_DRIVER::RESUMESEQUENCEREQUEST")
  (make-package "KORTEX_DRIVER::RESUMESEQUENCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESUMESEQUENCERESPONSE")
  (make-package "KORTEX_DRIVER::RESUMESEQUENCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResumeSequenceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResumeSequenceRequest
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

(defclass kortex_driver::ResumeSequenceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResumeSequenceResponse
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

(defclass kortex_driver::ResumeSequence
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResumeSequence :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResumeSequence :datatype-) "kortex_driver/ResumeSequence")
(setf (get kortex_driver::ResumeSequence :request) kortex_driver::ResumeSequenceRequest)
(setf (get kortex_driver::ResumeSequence :response) kortex_driver::ResumeSequenceResponse)

(defmethod kortex_driver::ResumeSequenceRequest
  (:response () (instance kortex_driver::ResumeSequenceResponse :init)))

(setf (get kortex_driver::ResumeSequenceRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResumeSequenceRequest :datatype-) "kortex_driver/ResumeSequenceRequest")
(setf (get kortex_driver::ResumeSequenceRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ResumeSequenceResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResumeSequenceResponse :datatype-) "kortex_driver/ResumeSequenceResponse")
(setf (get kortex_driver::ResumeSequenceResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ResumeSequence "f335b819dc59099fe3124b36f140ad07")


