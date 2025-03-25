;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlaySequence)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlaySequence (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYSEQUENCE")
  (make-package "KORTEX_DRIVER::PLAYSEQUENCE"))
(unless (find-package "KORTEX_DRIVER::PLAYSEQUENCEREQUEST")
  (make-package "KORTEX_DRIVER::PLAYSEQUENCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYSEQUENCERESPONSE")
  (make-package "KORTEX_DRIVER::PLAYSEQUENCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlaySequenceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlaySequenceRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SequenceHandle :init))
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
    ;; kortex_driver/SequenceHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlaySequenceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlaySequenceResponse
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

(defclass kortex_driver::PlaySequence
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlaySequence :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::PlaySequence :datatype-) "kortex_driver/PlaySequence")
(setf (get kortex_driver::PlaySequence :request) kortex_driver::PlaySequenceRequest)
(setf (get kortex_driver::PlaySequence :response) kortex_driver::PlaySequenceResponse)

(defmethod kortex_driver::PlaySequenceRequest
  (:response () (instance kortex_driver::PlaySequenceResponse :init)))

(setf (get kortex_driver::PlaySequenceRequest :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::PlaySequenceRequest :datatype-) "kortex_driver/PlaySequenceRequest")
(setf (get kortex_driver::PlaySequenceRequest :definition-)
      "SequenceHandle input

================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PlaySequenceResponse :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::PlaySequenceResponse :datatype-) "kortex_driver/PlaySequenceResponse")
(setf (get kortex_driver::PlaySequenceResponse :definition-)
      "SequenceHandle input

================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PlaySequence "04dfaeca45772f660e0913aa84774e13")


