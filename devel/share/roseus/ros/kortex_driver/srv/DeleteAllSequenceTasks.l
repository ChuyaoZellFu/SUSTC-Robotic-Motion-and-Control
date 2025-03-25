;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteAllSequenceTasks)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteAllSequenceTasks (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETEALLSEQUENCETASKS")
  (make-package "KORTEX_DRIVER::DELETEALLSEQUENCETASKS"))
(unless (find-package "KORTEX_DRIVER::DELETEALLSEQUENCETASKSREQUEST")
  (make-package "KORTEX_DRIVER::DELETEALLSEQUENCETASKSREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETEALLSEQUENCETASKSRESPONSE")
  (make-package "KORTEX_DRIVER::DELETEALLSEQUENCETASKSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteAllSequenceTasksRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteAllSequenceTasksRequest
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

(defclass kortex_driver::DeleteAllSequenceTasksResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteAllSequenceTasksResponse
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

(defclass kortex_driver::DeleteAllSequenceTasks
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteAllSequenceTasks :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteAllSequenceTasks :datatype-) "kortex_driver/DeleteAllSequenceTasks")
(setf (get kortex_driver::DeleteAllSequenceTasks :request) kortex_driver::DeleteAllSequenceTasksRequest)
(setf (get kortex_driver::DeleteAllSequenceTasks :response) kortex_driver::DeleteAllSequenceTasksResponse)

(defmethod kortex_driver::DeleteAllSequenceTasksRequest
  (:response () (instance kortex_driver::DeleteAllSequenceTasksResponse :init)))

(setf (get kortex_driver::DeleteAllSequenceTasksRequest :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteAllSequenceTasksRequest :datatype-) "kortex_driver/DeleteAllSequenceTasksRequest")
(setf (get kortex_driver::DeleteAllSequenceTasksRequest :definition-)
      "SequenceHandle input

================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteAllSequenceTasksResponse :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteAllSequenceTasksResponse :datatype-) "kortex_driver/DeleteAllSequenceTasksResponse")
(setf (get kortex_driver::DeleteAllSequenceTasksResponse :definition-)
      "SequenceHandle input

================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteAllSequenceTasks "04dfaeca45772f660e0913aa84774e13")


