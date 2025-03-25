;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Timestamp)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Timestamp (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TIMESTAMP")
  (make-package "KORTEX_DRIVER::TIMESTAMP"))

(in-package "ROS")
;;//! \htmlinclude Timestamp.msg.html


(defclass kortex_driver::Timestamp
  :super ros::object
  :slots (_sec _usec ))

(defmethod kortex_driver::Timestamp
  (:init
   (&key
    ((:sec __sec) 0)
    ((:usec __usec) 0)
    )
   (send-super :init)
   (setq _sec (round __sec))
   (setq _usec (round __usec))
   self)
  (:sec
   (&optional __sec)
   (if __sec (setq _sec __sec)) _sec)
  (:usec
   (&optional __usec)
   (if __usec (setq _usec __usec)) _usec)
  (:serialization-length
   ()
   (+
    ;; uint32 _sec
    4
    ;; uint32 _usec
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sec
       (write-long _sec s)
     ;; uint32 _usec
       (write-long _usec s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sec
     (setq _sec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _usec
     (setq _usec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Timestamp :md5sum-) "90da89d9efe7e712be65708e3d776fa4")
(setf (get kortex_driver::Timestamp :datatype-) "kortex_driver/Timestamp")
(setf (get kortex_driver::Timestamp :definition-)
      "
uint32 sec
uint32 usec
")



(provide :kortex_driver/Timestamp "90da89d9efe7e712be65708e3d776fa4")


