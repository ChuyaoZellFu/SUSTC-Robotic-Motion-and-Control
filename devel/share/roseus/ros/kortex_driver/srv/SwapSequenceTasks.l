;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SwapSequenceTasks)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SwapSequenceTasks (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SWAPSEQUENCETASKS")
  (make-package "KORTEX_DRIVER::SWAPSEQUENCETASKS"))
(unless (find-package "KORTEX_DRIVER::SWAPSEQUENCETASKSREQUEST")
  (make-package "KORTEX_DRIVER::SWAPSEQUENCETASKSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SWAPSEQUENCETASKSRESPONSE")
  (make-package "KORTEX_DRIVER::SWAPSEQUENCETASKSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SwapSequenceTasksRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SwapSequenceTasksRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SequenceTasksPair :init))
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
    ;; kortex_driver/SequenceTasksPair _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceTasksPair _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceTasksPair _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SwapSequenceTasksResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SwapSequenceTasksResponse
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

(defclass kortex_driver::SwapSequenceTasks
  :super ros::object
  :slots ())

(setf (get kortex_driver::SwapSequenceTasks :md5sum-) "5e966d67ad7cb2fd416e98745a3e8271")
(setf (get kortex_driver::SwapSequenceTasks :datatype-) "kortex_driver/SwapSequenceTasks")
(setf (get kortex_driver::SwapSequenceTasks :request) kortex_driver::SwapSequenceTasksRequest)
(setf (get kortex_driver::SwapSequenceTasks :response) kortex_driver::SwapSequenceTasksResponse)

(defmethod kortex_driver::SwapSequenceTasksRequest
  (:response () (instance kortex_driver::SwapSequenceTasksResponse :init)))

(setf (get kortex_driver::SwapSequenceTasksRequest :md5sum-) "5e966d67ad7cb2fd416e98745a3e8271")
(setf (get kortex_driver::SwapSequenceTasksRequest :datatype-) "kortex_driver/SwapSequenceTasksRequest")
(setf (get kortex_driver::SwapSequenceTasksRequest :definition-)
      "SequenceTasksPair input

================================================================================
MSG: kortex_driver/SequenceTasksPair

SequenceHandle sequence_handle
uint32 first_task_index
uint32 second_task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SwapSequenceTasksResponse :md5sum-) "5e966d67ad7cb2fd416e98745a3e8271")
(setf (get kortex_driver::SwapSequenceTasksResponse :datatype-) "kortex_driver/SwapSequenceTasksResponse")
(setf (get kortex_driver::SwapSequenceTasksResponse :definition-)
      "SequenceTasksPair input

================================================================================
MSG: kortex_driver/SequenceTasksPair

SequenceHandle sequence_handle
uint32 first_task_index
uint32 second_task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SwapSequenceTasks "5e966d67ad7cb2fd416e98745a3e8271")


