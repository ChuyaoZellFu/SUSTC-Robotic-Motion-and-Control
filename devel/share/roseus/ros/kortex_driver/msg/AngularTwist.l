;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::AngularTwist)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'AngularTwist (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ANGULARTWIST")
  (make-package "KORTEX_DRIVER::ANGULARTWIST"))

(in-package "ROS")
;;//! \htmlinclude AngularTwist.msg.html


(defclass kortex_driver::AngularTwist
  :super ros::object
  :slots (_angular ))

(defmethod kortex_driver::AngularTwist
  (:init
   (&key
    ((:angular __angular) 0.0)
    )
   (send-super :init)
   (setq _angular (float __angular))
   self)
  (:angular
   (&optional __angular)
   (if __angular (setq _angular __angular)) _angular)
  (:serialization-length
   ()
   (+
    ;; float32 _angular
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angular
       (sys::poke _angular (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angular
     (setq _angular (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::AngularTwist :md5sum-) "960aa4f2aaeac14706e7fa1a905e8a00")
(setf (get kortex_driver::AngularTwist :datatype-) "kortex_driver/AngularTwist")
(setf (get kortex_driver::AngularTwist :definition-)
      "
float32 angular
")



(provide :kortex_driver/AngularTwist "960aa4f2aaeac14706e7fa1a905e8a00")


