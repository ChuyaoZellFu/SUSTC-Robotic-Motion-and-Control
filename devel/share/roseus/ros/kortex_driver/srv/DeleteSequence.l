;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteSequence)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteSequence (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETESEQUENCE")
  (make-package "KORTEX_DRIVER::DELETESEQUENCE"))
(unless (find-package "KORTEX_DRIVER::DELETESEQUENCEREQUEST")
  (make-package "KORTEX_DRIVER::DELETESEQUENCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETESEQUENCERESPONSE")
  (make-package "KORTEX_DRIVER::DELETESEQUENCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteSequenceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteSequenceRequest
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

(defclass kortex_driver::DeleteSequenceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteSequenceResponse
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

(defclass kortex_driver::DeleteSequence
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteSequence :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteSequence :datatype-) "kortex_driver/DeleteSequence")
(setf (get kortex_driver::DeleteSequence :request) kortex_driver::DeleteSequenceRequest)
(setf (get kortex_driver::DeleteSequence :response) kortex_driver::DeleteSequenceResponse)

(defmethod kortex_driver::DeleteSequenceRequest
  (:response () (instance kortex_driver::DeleteSequenceResponse :init)))

(setf (get kortex_driver::DeleteSequenceRequest :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteSequenceRequest :datatype-) "kortex_driver/DeleteSequenceRequest")
(setf (get kortex_driver::DeleteSequenceRequest :definition-)
      "SequenceHandle input

================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteSequenceResponse :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteSequenceResponse :datatype-) "kortex_driver/DeleteSequenceResponse")
(setf (get kortex_driver::DeleteSequenceResponse :definition-)
      "SequenceHandle input

================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteSequence "04dfaeca45772f660e0913aa84774e13")


