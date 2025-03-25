;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TrajectoryErrorElement)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TrajectoryErrorElement (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRAJECTORYERRORELEMENT")
  (make-package "KORTEX_DRIVER::TRAJECTORYERRORELEMENT"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryErrorElement.msg.html


(defclass kortex_driver::TrajectoryErrorElement
  :super ros::object
  :slots (_error_type _error_identifier _error_value _min_value _max_value _index _message _waypoint_index ))

(defmethod kortex_driver::TrajectoryErrorElement
  (:init
   (&key
    ((:error_type __error_type) 0)
    ((:error_identifier __error_identifier) 0)
    ((:error_value __error_value) 0.0)
    ((:min_value __min_value) 0.0)
    ((:max_value __max_value) 0.0)
    ((:index __index) 0)
    ((:message __message) "")
    ((:waypoint_index __waypoint_index) 0)
    )
   (send-super :init)
   (setq _error_type (round __error_type))
   (setq _error_identifier (round __error_identifier))
   (setq _error_value (float __error_value))
   (setq _min_value (float __min_value))
   (setq _max_value (float __max_value))
   (setq _index (round __index))
   (setq _message (string __message))
   (setq _waypoint_index (round __waypoint_index))
   self)
  (:error_type
   (&optional __error_type)
   (if __error_type (setq _error_type __error_type)) _error_type)
  (:error_identifier
   (&optional __error_identifier)
   (if __error_identifier (setq _error_identifier __error_identifier)) _error_identifier)
  (:error_value
   (&optional __error_value)
   (if __error_value (setq _error_value __error_value)) _error_value)
  (:min_value
   (&optional __min_value)
   (if __min_value (setq _min_value __min_value)) _min_value)
  (:max_value
   (&optional __max_value)
   (if __max_value (setq _max_value __max_value)) _max_value)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:waypoint_index
   (&optional __waypoint_index)
   (if __waypoint_index (setq _waypoint_index __waypoint_index)) _waypoint_index)
  (:serialization-length
   ()
   (+
    ;; uint32 _error_type
    4
    ;; uint32 _error_identifier
    4
    ;; float32 _error_value
    4
    ;; float32 _min_value
    4
    ;; float32 _max_value
    4
    ;; uint32 _index
    4
    ;; string _message
    4 (length _message)
    ;; uint32 _waypoint_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _error_type
       (write-long _error_type s)
     ;; uint32 _error_identifier
       (write-long _error_identifier s)
     ;; float32 _error_value
       (sys::poke _error_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_value
       (sys::poke _min_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_value
       (sys::poke _max_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _index
       (write-long _index s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; uint32 _waypoint_index
       (write-long _waypoint_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _error_type
     (setq _error_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _error_identifier
     (setq _error_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _error_value
     (setq _error_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_value
     (setq _min_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_value
     (setq _max_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _waypoint_index
     (setq _waypoint_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TrajectoryErrorElement :md5sum-) "fe19f45de474fe8a7f3de643ea88e472")
(setf (get kortex_driver::TrajectoryErrorElement :datatype-) "kortex_driver/TrajectoryErrorElement")
(setf (get kortex_driver::TrajectoryErrorElement :definition-)
      "
uint32 error_type
uint32 error_identifier
float32 error_value
float32 min_value
float32 max_value
uint32 index
string message
uint32 waypoint_index
")



(provide :kortex_driver/TrajectoryErrorElement "fe19f45de474fe8a7f3de643ea88e472")


