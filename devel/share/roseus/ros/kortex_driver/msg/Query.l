;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Query)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Query (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::QUERY")
  (make-package "KORTEX_DRIVER::QUERY"))

(in-package "ROS")
;;//! \htmlinclude Query.msg.html


(defclass kortex_driver::Query
  :super ros::object
  :slots (_start_timestamp _end_timestamp _username ))

(defmethod kortex_driver::Query
  (:init
   (&key
    ((:start_timestamp __start_timestamp) (instance kortex_driver::Timestamp :init))
    ((:end_timestamp __end_timestamp) (instance kortex_driver::Timestamp :init))
    ((:username __username) "")
    )
   (send-super :init)
   (setq _start_timestamp __start_timestamp)
   (setq _end_timestamp __end_timestamp)
   (setq _username (string __username))
   self)
  (:start_timestamp
   (&rest __start_timestamp)
   (if (keywordp (car __start_timestamp))
       (send* _start_timestamp __start_timestamp)
     (progn
       (if __start_timestamp (setq _start_timestamp (car __start_timestamp)))
       _start_timestamp)))
  (:end_timestamp
   (&rest __end_timestamp)
   (if (keywordp (car __end_timestamp))
       (send* _end_timestamp __end_timestamp)
     (progn
       (if __end_timestamp (setq _end_timestamp (car __end_timestamp)))
       _end_timestamp)))
  (:username
   (&optional __username)
   (if __username (setq _username __username)) _username)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Timestamp _start_timestamp
    (send _start_timestamp :serialization-length)
    ;; kortex_driver/Timestamp _end_timestamp
    (send _end_timestamp :serialization-length)
    ;; string _username
    4 (length _username)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Timestamp _start_timestamp
       (send _start_timestamp :serialize s)
     ;; kortex_driver/Timestamp _end_timestamp
       (send _end_timestamp :serialize s)
     ;; string _username
       (write-long (length _username) s) (princ _username s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Timestamp _start_timestamp
     (send _start_timestamp :deserialize buf ptr-) (incf ptr- (send _start_timestamp :serialization-length))
   ;; kortex_driver/Timestamp _end_timestamp
     (send _end_timestamp :deserialize buf ptr-) (incf ptr- (send _end_timestamp :serialization-length))
   ;; string _username
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _username (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::Query :md5sum-) "13ee5e330f26cd5fd95364d08fb2a910")
(setf (get kortex_driver::Query :datatype-) "kortex_driver/Query")
(setf (get kortex_driver::Query :definition-)
      "
Timestamp start_timestamp
Timestamp end_timestamp
string username
================================================================================
MSG: kortex_driver/Timestamp

uint32 sec
uint32 usec
")



(provide :kortex_driver/Query "13ee5e330f26cd5fd95364d08fb2a910")


