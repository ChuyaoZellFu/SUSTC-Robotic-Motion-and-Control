;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_Stop)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_Stop (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_STOP")
  (make-package "KORTEX_DRIVER::BASE_STOP"))

(in-package "ROS")
;;//! \htmlinclude Base_Stop.msg.html


(defclass kortex_driver::Base_Stop
  :super ros::object
  :slots ())

(defmethod kortex_driver::Base_Stop
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

(setf (get kortex_driver::Base_Stop :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get kortex_driver::Base_Stop :datatype-) "kortex_driver/Base_Stop")
(setf (get kortex_driver::Base_Stop :definition-)
      "
")



(provide :kortex_driver/Base_Stop "d41d8cd98f00b204e9800998ecf8427e")


