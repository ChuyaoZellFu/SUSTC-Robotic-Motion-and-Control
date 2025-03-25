;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TwistLinearSoftLimit)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TwistLinearSoftLimit (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TWISTLINEARSOFTLIMIT")
  (make-package "KORTEX_DRIVER::TWISTLINEARSOFTLIMIT"))

(in-package "ROS")
;;//! \htmlinclude TwistLinearSoftLimit.msg.html


(defclass kortex_driver::TwistLinearSoftLimit
  :super ros::object
  :slots (_control_mode _twist_linear_soft_limit ))

(defmethod kortex_driver::TwistLinearSoftLimit
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    ((:twist_linear_soft_limit __twist_linear_soft_limit) 0.0)
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   (setq _twist_linear_soft_limit (float __twist_linear_soft_limit))
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:twist_linear_soft_limit
   (&optional __twist_linear_soft_limit)
   (if __twist_linear_soft_limit (setq _twist_linear_soft_limit __twist_linear_soft_limit)) _twist_linear_soft_limit)
  (:serialization-length
   ()
   (+
    ;; uint32 _control_mode
    4
    ;; float32 _twist_linear_soft_limit
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _control_mode
       (write-long _control_mode s)
     ;; float32 _twist_linear_soft_limit
       (sys::poke _twist_linear_soft_limit (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _control_mode
     (setq _control_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _twist_linear_soft_limit
     (setq _twist_linear_soft_limit (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TwistLinearSoftLimit :md5sum-) "56b2076ba48a21a3287444da3b0c4e3c")
(setf (get kortex_driver::TwistLinearSoftLimit :datatype-) "kortex_driver/TwistLinearSoftLimit")
(setf (get kortex_driver::TwistLinearSoftLimit :definition-)
      "
uint32 control_mode
float32 twist_linear_soft_limit
")



(provide :kortex_driver/TwistLinearSoftLimit "56b2076ba48a21a3287444da3b0c4e3c")


