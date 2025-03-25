;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::LinearTwist)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'LinearTwist (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::LINEARTWIST")
  (make-package "KORTEX_DRIVER::LINEARTWIST"))

(in-package "ROS")
;;//! \htmlinclude LinearTwist.msg.html


(defclass kortex_driver::LinearTwist
  :super ros::object
  :slots (_linear ))

(defmethod kortex_driver::LinearTwist
  (:init
   (&key
    ((:linear __linear) 0.0)
    )
   (send-super :init)
   (setq _linear (float __linear))
   self)
  (:linear
   (&optional __linear)
   (if __linear (setq _linear __linear)) _linear)
  (:serialization-length
   ()
   (+
    ;; float32 _linear
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _linear
       (sys::poke _linear (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _linear
     (setq _linear (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::LinearTwist :md5sum-) "aeedbad760b28cbccafc1668ae4ed22c")
(setf (get kortex_driver::LinearTwist :datatype-) "kortex_driver/LinearTwist")
(setf (get kortex_driver::LinearTwist :definition-)
      "
float32 linear
")



(provide :kortex_driver/LinearTwist "aeedbad760b28cbccafc1668ae4ed22c")


