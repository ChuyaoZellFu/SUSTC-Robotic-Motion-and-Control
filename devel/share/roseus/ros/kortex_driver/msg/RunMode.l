;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RunMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RunMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RUNMODE")
  (make-package "KORTEX_DRIVER::RUNMODE"))

(in-package "ROS")
;;//! \htmlinclude RunMode.msg.html


(defclass kortex_driver::RunMode
  :super ros::object
  :slots (_run_mode ))

(defmethod kortex_driver::RunMode
  (:init
   (&key
    ((:run_mode __run_mode) 0)
    )
   (send-super :init)
   (setq _run_mode (round __run_mode))
   self)
  (:run_mode
   (&optional __run_mode)
   (if __run_mode (setq _run_mode __run_mode)) _run_mode)
  (:serialization-length
   ()
   (+
    ;; uint32 _run_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _run_mode
       (write-long _run_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _run_mode
     (setq _run_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::RunMode :md5sum-) "abe2eb774be1b28bf7c30a06ea5e3691")
(setf (get kortex_driver::RunMode :datatype-) "kortex_driver/RunMode")
(setf (get kortex_driver::RunMode :definition-)
      "
uint32 run_mode
")



(provide :kortex_driver/RunMode "abe2eb774be1b28bf7c30a06ea5e3691")


