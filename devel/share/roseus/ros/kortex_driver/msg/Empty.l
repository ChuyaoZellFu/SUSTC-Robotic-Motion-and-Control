;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Empty)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Empty (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::EMPTY")
  (make-package "KORTEX_DRIVER::EMPTY"))

(in-package "ROS")
;;//! \htmlinclude Empty.msg.html


(defclass kortex_driver::Empty
  :super ros::object
  :slots ())

(defmethod kortex_driver::Empty
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::Empty :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get kortex_driver::Empty :datatype-) "kortex_driver/Empty")
(setf (get kortex_driver::Empty :definition-)
      "
")



(provide :kortex_driver/Empty "d41d8cd98f00b204e9800998ecf8427e")


