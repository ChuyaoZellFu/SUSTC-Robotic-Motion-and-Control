;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SystemTime)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SystemTime (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SYSTEMTIME")
  (make-package "KORTEX_DRIVER::SYSTEMTIME"))

(in-package "ROS")
;;//! \htmlinclude SystemTime.msg.html


(defclass kortex_driver::SystemTime
  :super ros::object
  :slots (_sec _min _hour _mday _mon _year ))

(defmethod kortex_driver::SystemTime
  (:init
   (&key
    ((:sec __sec) 0)
    ((:min __min) 0)
    ((:hour __hour) 0)
    ((:mday __mday) 0)
    ((:mon __mon) 0)
    ((:year __year) 0)
    )
   (send-super :init)
   (setq _sec (round __sec))
   (setq _min (round __min))
   (setq _hour (round __hour))
   (setq _mday (round __mday))
   (setq _mon (round __mon))
   (setq _year (round __year))
   self)
  (:sec
   (&optional __sec)
   (if __sec (setq _sec __sec)) _sec)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:hour
   (&optional __hour)
   (if __hour (setq _hour __hour)) _hour)
  (:mday
   (&optional __mday)
   (if __mday (setq _mday __mday)) _mday)
  (:mon
   (&optional __mon)
   (if __mon (setq _mon __mon)) _mon)
  (:year
   (&optional __year)
   (if __year (setq _year __year)) _year)
  (:serialization-length
   ()
   (+
    ;; uint32 _sec
    4
    ;; uint32 _min
    4
    ;; uint32 _hour
    4
    ;; uint32 _mday
    4
    ;; uint32 _mon
    4
    ;; uint32 _year
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sec
       (write-long _sec s)
     ;; uint32 _min
       (write-long _min s)
     ;; uint32 _hour
       (write-long _hour s)
     ;; uint32 _mday
       (write-long _mday s)
     ;; uint32 _mon
       (write-long _mon s)
     ;; uint32 _year
       (write-long _year s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sec
     (setq _sec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _min
     (setq _min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _hour
     (setq _hour (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _mday
     (setq _mday (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _mon
     (setq _mon (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _year
     (setq _year (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SystemTime :md5sum-) "c3c3b4e6f22b647897641173820f05db")
(setf (get kortex_driver::SystemTime :datatype-) "kortex_driver/SystemTime")
(setf (get kortex_driver::SystemTime :definition-)
      "
uint32 sec
uint32 min
uint32 hour
uint32 mday
uint32 mon
uint32 year
")



(provide :kortex_driver/SystemTime "c3c3b4e6f22b647897641173820f05db")


