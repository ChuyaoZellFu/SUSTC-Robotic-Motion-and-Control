;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SequenceTasksRange)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SequenceTasksRange (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SEQUENCETASKSRANGE")
  (make-package "KORTEX_DRIVER::SEQUENCETASKSRANGE"))

(in-package "ROS")
;;//! \htmlinclude SequenceTasksRange.msg.html


(defclass kortex_driver::SequenceTasksRange
  :super ros::object
  :slots (_first_task_index _second_task_index ))

(defmethod kortex_driver::SequenceTasksRange
  (:init
   (&key
    ((:first_task_index __first_task_index) 0)
    ((:second_task_index __second_task_index) 0)
    )
   (send-super :init)
   (setq _first_task_index (round __first_task_index))
   (setq _second_task_index (round __second_task_index))
   self)
  (:first_task_index
   (&optional __first_task_index)
   (if __first_task_index (setq _first_task_index __first_task_index)) _first_task_index)
  (:second_task_index
   (&optional __second_task_index)
   (if __second_task_index (setq _second_task_index __second_task_index)) _second_task_index)
  (:serialization-length
   ()
   (+
    ;; uint32 _first_task_index
    4
    ;; uint32 _second_task_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _first_task_index
       (write-long _first_task_index s)
     ;; uint32 _second_task_index
       (write-long _second_task_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _first_task_index
     (setq _first_task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _second_task_index
     (setq _second_task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SequenceTasksRange :md5sum-) "f9dba2f68de4791eab54435a0c4b312c")
(setf (get kortex_driver::SequenceTasksRange :datatype-) "kortex_driver/SequenceTasksRange")
(setf (get kortex_driver::SequenceTasksRange :definition-)
      "
uint32 first_task_index
uint32 second_task_index
")



(provide :kortex_driver/SequenceTasksRange "f9dba2f68de4791eab54435a0c4b312c")


