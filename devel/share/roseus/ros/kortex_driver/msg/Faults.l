;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Faults)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Faults (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FAULTS")
  (make-package "KORTEX_DRIVER::FAULTS"))

(in-package "ROS")
;;//! \htmlinclude Faults.msg.html


(defclass kortex_driver::Faults
  :super ros::object
  :slots ())

(defmethod kortex_driver::Faults
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

(setf (get kortex_driver::Faults :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get kortex_driver::Faults :datatype-) "kortex_driver/Faults")
(setf (get kortex_driver::Faults :definition-)
      "
")



(provide :kortex_driver/Faults "d41d8cd98f00b204e9800998ecf8427e")


