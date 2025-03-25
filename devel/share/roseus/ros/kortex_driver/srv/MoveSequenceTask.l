;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MoveSequenceTask)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MoveSequenceTask (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MOVESEQUENCETASK")
  (make-package "KORTEX_DRIVER::MOVESEQUENCETASK"))
(unless (find-package "KORTEX_DRIVER::MOVESEQUENCETASKREQUEST")
  (make-package "KORTEX_DRIVER::MOVESEQUENCETASKREQUEST"))
(unless (find-package "KORTEX_DRIVER::MOVESEQUENCETASKRESPONSE")
  (make-package "KORTEX_DRIVER::MOVESEQUENCETASKRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::MoveSequenceTaskRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::MoveSequenceTaskRequest
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

(defclass kortex_driver::MoveSequenceTaskResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::MoveSequenceTaskResponse
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

(defclass kortex_driver::MoveSequenceTask
  :super ros::object
  :slots ())

(setf (get kortex_driver::MoveSequenceTask :md5sum-) "5e966d67ad7cb2fd416e98745a3e8271")
(setf (get kortex_driver::MoveSequenceTask :datatype-) "kortex_driver/MoveSequenceTask")
(setf (get kortex_driver::MoveSequenceTask :request) kortex_driver::MoveSequenceTaskRequest)
(setf (get kortex_driver::MoveSequenceTask :response) kortex_driver::MoveSequenceTaskResponse)

(defmethod kortex_driver::MoveSequenceTaskRequest
  (:response () (instance kortex_driver::MoveSequenceTaskResponse :init)))

(setf (get kortex_driver::MoveSequenceTaskRequest :md5sum-) "5e966d67ad7cb2fd416e98745a3e8271")
(setf (get kortex_driver::MoveSequenceTaskRequest :datatype-) "kortex_driver/MoveSequenceTaskRequest")
(setf (get kortex_driver::MoveSequenceTaskRequest :definition-)
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

(setf (get kortex_driver::MoveSequenceTaskResponse :md5sum-) "5e966d67ad7cb2fd416e98745a3e8271")
(setf (get kortex_driver::MoveSequenceTaskResponse :datatype-) "kortex_driver/MoveSequenceTaskResponse")
(setf (get kortex_driver::MoveSequenceTaskResponse :definition-)
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



(provide :kortex_driver/MoveSequenceTask "5e966d67ad7cb2fd416e98745a3e8271")


