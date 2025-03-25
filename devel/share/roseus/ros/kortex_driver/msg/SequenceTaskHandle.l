;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SequenceTaskHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SequenceTaskHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SEQUENCETASKHANDLE")
  (make-package "KORTEX_DRIVER::SEQUENCETASKHANDLE"))

(in-package "ROS")
;;//! \htmlinclude SequenceTaskHandle.msg.html


(defclass kortex_driver::SequenceTaskHandle
  :super ros::object
  :slots (_sequence_handle _task_index ))

(defmethod kortex_driver::SequenceTaskHandle
  (:init
   (&key
    ((:sequence_handle __sequence_handle) (instance kortex_driver::SequenceHandle :init))
    ((:task_index __task_index) 0)
    )
   (send-super :init)
   (setq _sequence_handle __sequence_handle)
   (setq _task_index (round __task_index))
   self)
  (:sequence_handle
   (&rest __sequence_handle)
   (if (keywordp (car __sequence_handle))
       (send* _sequence_handle __sequence_handle)
     (progn
       (if __sequence_handle (setq _sequence_handle (car __sequence_handle)))
       _sequence_handle)))
  (:task_index
   (&optional __task_index)
   (if __task_index (setq _task_index __task_index)) _task_index)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SequenceHandle _sequence_handle
    (send _sequence_handle :serialization-length)
    ;; uint32 _task_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceHandle _sequence_handle
       (send _sequence_handle :serialize s)
     ;; uint32 _task_index
       (write-long _task_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceHandle _sequence_handle
     (send _sequence_handle :deserialize buf ptr-) (incf ptr- (send _sequence_handle :serialization-length))
   ;; uint32 _task_index
     (setq _task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SequenceTaskHandle :md5sum-) "d86cef527dffeac4930486133afaaef1")
(setf (get kortex_driver::SequenceTaskHandle :datatype-) "kortex_driver/SequenceTaskHandle")
(setf (get kortex_driver::SequenceTaskHandle :definition-)
      "
SequenceHandle sequence_handle
uint32 task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/SequenceTaskHandle "d86cef527dffeac4930486133afaaef1")


