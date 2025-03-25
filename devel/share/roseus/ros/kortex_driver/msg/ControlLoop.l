;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlLoop)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlLoop (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLOOP")
  (make-package "KORTEX_DRIVER::CONTROLLOOP"))

(in-package "ROS")
;;//! \htmlinclude ControlLoop.msg.html


(defclass kortex_driver::ControlLoop
  :super ros::object
  :slots (_control_loop ))

(defmethod kortex_driver::ControlLoop
  (:init
   (&key
    ((:control_loop __control_loop) 0)
    )
   (send-super :init)
   (setq _control_loop (round __control_loop))
   self)
  (:control_loop
   (&optional __control_loop)
   (if __control_loop (setq _control_loop __control_loop)) _control_loop)
  (:serialization-length
   ()
   (+
    ;; uint32 _control_loop
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _control_loop
       (write-long _control_loop s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _control_loop
     (setq _control_loop (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ControlLoop :md5sum-) "26936d7a4ae5672d376ef90225be07d4")
(setf (get kortex_driver::ControlLoop :datatype-) "kortex_driver/ControlLoop")
(setf (get kortex_driver::ControlLoop :definition-)
      "
uint32 control_loop
")



(provide :kortex_driver/ControlLoop "26936d7a4ae5672d376ef90225be07d4")


