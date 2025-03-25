;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteSequenceTask)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteSequenceTask (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETESEQUENCETASK")
  (make-package "KORTEX_DRIVER::DELETESEQUENCETASK"))
(unless (find-package "KORTEX_DRIVER::DELETESEQUENCETASKREQUEST")
  (make-package "KORTEX_DRIVER::DELETESEQUENCETASKREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETESEQUENCETASKRESPONSE")
  (make-package "KORTEX_DRIVER::DELETESEQUENCETASKRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteSequenceTaskRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteSequenceTaskRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SequenceTaskHandle :init))
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
    ;; kortex_driver/SequenceTaskHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceTaskHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceTaskHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DeleteSequenceTaskResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteSequenceTaskResponse
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

(defclass kortex_driver::DeleteSequenceTask
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteSequenceTask :md5sum-) "830e67a103da79a18d6fa6ffc8b42b46")
(setf (get kortex_driver::DeleteSequenceTask :datatype-) "kortex_driver/DeleteSequenceTask")
(setf (get kortex_driver::DeleteSequenceTask :request) kortex_driver::DeleteSequenceTaskRequest)
(setf (get kortex_driver::DeleteSequenceTask :response) kortex_driver::DeleteSequenceTaskResponse)

(defmethod kortex_driver::DeleteSequenceTaskRequest
  (:response () (instance kortex_driver::DeleteSequenceTaskResponse :init)))

(setf (get kortex_driver::DeleteSequenceTaskRequest :md5sum-) "830e67a103da79a18d6fa6ffc8b42b46")
(setf (get kortex_driver::DeleteSequenceTaskRequest :datatype-) "kortex_driver/DeleteSequenceTaskRequest")
(setf (get kortex_driver::DeleteSequenceTaskRequest :definition-)
      "SequenceTaskHandle input

================================================================================
MSG: kortex_driver/SequenceTaskHandle

SequenceHandle sequence_handle
uint32 task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteSequenceTaskResponse :md5sum-) "830e67a103da79a18d6fa6ffc8b42b46")
(setf (get kortex_driver::DeleteSequenceTaskResponse :datatype-) "kortex_driver/DeleteSequenceTaskResponse")
(setf (get kortex_driver::DeleteSequenceTaskResponse :definition-)
      "SequenceTaskHandle input

================================================================================
MSG: kortex_driver/SequenceTaskHandle

SequenceHandle sequence_handle
uint32 task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteSequenceTask "830e67a103da79a18d6fa6ffc8b42b46")


