;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Delay)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Delay (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELAY")
  (make-package "KORTEX_DRIVER::DELAY"))

(in-package "ROS")
;;//! \htmlinclude Delay.msg.html


(defclass kortex_driver::Delay
  :super ros::object
  :slots (_duration ))

(defmethod kortex_driver::Delay
  (:init
   (&key
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _duration (round __duration))
   self)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Delay :md5sum-) "eea8d32b4658822ac77c4c94fbbb5f2b")
(setf (get kortex_driver::Delay :datatype-) "kortex_driver/Delay")
(setf (get kortex_driver::Delay :definition-)
      "
uint32 duration
")



(provide :kortex_driver/Delay "eea8d32b4658822ac77c4c94fbbb5f2b")


