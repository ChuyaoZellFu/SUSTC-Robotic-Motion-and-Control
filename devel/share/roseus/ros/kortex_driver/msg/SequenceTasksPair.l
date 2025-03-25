;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SequenceTasksPair)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SequenceTasksPair (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SEQUENCETASKSPAIR")
  (make-package "KORTEX_DRIVER::SEQUENCETASKSPAIR"))

(in-package "ROS")
;;//! \htmlinclude SequenceTasksPair.msg.html


(defclass kortex_driver::SequenceTasksPair
  :super ros::object
  :slots (_sequence_handle _first_task_index _second_task_index ))

(defmethod kortex_driver::SequenceTasksPair
  (:init
   (&key
    ((:sequence_handle __sequence_handle) (instance kortex_driver::SequenceHandle :init))
    ((:first_task_index __first_task_index) 0)
    ((:second_task_index __second_task_index) 0)
    )
   (send-super :init)
   (setq _sequence_handle __sequence_handle)
   (setq _first_task_index (round __first_task_index))
   (setq _second_task_index (round __second_task_index))
   self)
  (:sequence_handle
   (&rest __sequence_handle)
   (if (keywordp (car __sequence_handle))
       (send* _sequence_handle __sequence_handle)
     (progn
       (if __sequence_handle (setq _sequence_handle (car __sequence_handle)))
       _sequence_handle)))
  (:first_task_index
   (&optional __first_task_index)
   (if __first_task_index (setq _first_task_index __first_task_index)) _first_task_index)
  (:second_task_index
   (&optional __second_task_index)
   (if __second_task_index (setq _second_task_index __second_task_index)) _second_task_index)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SequenceHandle _sequence_handle
    (send _sequence_handle :serialization-length)
    ;; uint32 _first_task_index
    4
    ;; uint32 _second_task_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceHandle _sequence_handle
       (send _sequence_handle :serialize s)
     ;; uint32 _first_task_index
       (write-long _first_task_index s)
     ;; uint32 _second_task_index
       (write-long _second_task_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceHandle _sequence_handle
     (send _sequence_handle :deserialize buf ptr-) (incf ptr- (send _sequence_handle :serialization-length))
   ;; uint32 _first_task_index
     (setq _first_task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _second_task_index
     (setq _second_task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SequenceTasksPair :md5sum-) "1e54820745915e6c8b6b14cf1311455d")
(setf (get kortex_driver::SequenceTasksPair :datatype-) "kortex_driver/SequenceTasksPair")
(setf (get kortex_driver::SequenceTasksPair :definition-)
      "
SequenceHandle sequence_handle
uint32 first_task_index
uint32 second_task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/SequenceTasksPair "1e54820745915e6c8b6b14cf1311455d")


