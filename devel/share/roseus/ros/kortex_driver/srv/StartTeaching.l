;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StartTeaching)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StartTeaching (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STARTTEACHING")
  (make-package "KORTEX_DRIVER::STARTTEACHING"))
(unless (find-package "KORTEX_DRIVER::STARTTEACHINGREQUEST")
  (make-package "KORTEX_DRIVER::STARTTEACHINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::STARTTEACHINGRESPONSE")
  (make-package "KORTEX_DRIVER::STARTTEACHINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::StartTeachingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::StartTeachingRequest
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

(defclass kortex_driver::StartTeachingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::StartTeachingResponse
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

(defclass kortex_driver::StartTeaching
  :super ros::object
  :slots ())

(setf (get kortex_driver::StartTeaching :md5sum-) "830e67a103da79a18d6fa6ffc8b42b46")
(setf (get kortex_driver::StartTeaching :datatype-) "kortex_driver/StartTeaching")
(setf (get kortex_driver::StartTeaching :request) kortex_driver::StartTeachingRequest)
(setf (get kortex_driver::StartTeaching :response) kortex_driver::StartTeachingResponse)

(defmethod kortex_driver::StartTeachingRequest
  (:response () (instance kortex_driver::StartTeachingResponse :init)))

(setf (get kortex_driver::StartTeachingRequest :md5sum-) "830e67a103da79a18d6fa6ffc8b42b46")
(setf (get kortex_driver::StartTeachingRequest :datatype-) "kortex_driver/StartTeachingRequest")
(setf (get kortex_driver::StartTeachingRequest :definition-)
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

(setf (get kortex_driver::StartTeachingResponse :md5sum-) "830e67a103da79a18d6fa6ffc8b42b46")
(setf (get kortex_driver::StartTeachingResponse :datatype-) "kortex_driver/StartTeachingResponse")
(setf (get kortex_driver::StartTeachingResponse :definition-)
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



(provide :kortex_driver/StartTeaching "830e67a103da79a18d6fa6ffc8b42b46")


