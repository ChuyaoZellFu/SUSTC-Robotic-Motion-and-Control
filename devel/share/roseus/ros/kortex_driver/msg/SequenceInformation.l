;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SequenceInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SequenceInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SEQUENCEINFORMATION")
  (make-package "KORTEX_DRIVER::SEQUENCEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude SequenceInformation.msg.html


(defclass kortex_driver::SequenceInformation
  :super ros::object
  :slots (_event_identifier _task_index _task_identifier ))

(defmethod kortex_driver::SequenceInformation
  (:init
   (&key
    ((:event_identifier __event_identifier) 0)
    ((:task_index __task_index) 0)
    ((:task_identifier __task_identifier) 0)
    )
   (send-super :init)
   (setq _event_identifier (round __event_identifier))
   (setq _task_index (round __task_index))
   (setq _task_identifier (round __task_identifier))
   self)
  (:event_identifier
   (&optional __event_identifier)
   (if __event_identifier (setq _event_identifier __event_identifier)) _event_identifier)
  (:task_index
   (&optional __task_index)
   (if __task_index (setq _task_index __task_index)) _task_index)
  (:task_identifier
   (&optional __task_identifier)
   (if __task_identifier (setq _task_identifier __task_identifier)) _task_identifier)
  (:serialization-length
   ()
   (+
    ;; uint32 _event_identifier
    4
    ;; uint32 _task_index
    4
    ;; uint32 _task_identifier
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _event_identifier
       (write-long _event_identifier s)
     ;; uint32 _task_index
       (write-long _task_index s)
     ;; uint32 _task_identifier
       (write-long _task_identifier s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _event_identifier
     (setq _event_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _task_index
     (setq _task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _task_identifier
     (setq _task_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SequenceInformation :md5sum-) "a7eedb18a251848053b5142e4934ee01")
(setf (get kortex_driver::SequenceInformation :datatype-) "kortex_driver/SequenceInformation")
(setf (get kortex_driver::SequenceInformation :definition-)
      "
uint32 event_identifier
uint32 task_index
uint32 task_identifier
")



(provide :kortex_driver/SequenceInformation "a7eedb18a251848053b5142e4934ee01")


