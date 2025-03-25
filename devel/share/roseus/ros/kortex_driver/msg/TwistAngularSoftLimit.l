;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TwistAngularSoftLimit)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TwistAngularSoftLimit (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TWISTANGULARSOFTLIMIT")
  (make-package "KORTEX_DRIVER::TWISTANGULARSOFTLIMIT"))

(in-package "ROS")
;;//! \htmlinclude TwistAngularSoftLimit.msg.html


(defclass kortex_driver::TwistAngularSoftLimit
  :super ros::object
  :slots (_control_mode _twist_angular_soft_limit ))

(defmethod kortex_driver::TwistAngularSoftLimit
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    ((:twist_angular_soft_limit __twist_angular_soft_limit) 0.0)
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   (setq _twist_angular_soft_limit (float __twist_angular_soft_limit))
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:twist_angular_soft_limit
   (&optional __twist_angular_soft_limit)
   (if __twist_angular_soft_limit (setq _twist_angular_soft_limit __twist_angular_soft_limit)) _twist_angular_soft_limit)
  (:serialization-length
   ()
   (+
    ;; uint32 _control_mode
    4
    ;; float32 _twist_angular_soft_limit
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _control_mode
       (write-long _control_mode s)
     ;; float32 _twist_angular_soft_limit
       (sys::poke _twist_angular_soft_limit (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _control_mode
     (setq _control_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _twist_angular_soft_limit
     (setq _twist_angular_soft_limit (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TwistAngularSoftLimit :md5sum-) "ec20d4ed4983e99d6172917731a2eee5")
(setf (get kortex_driver::TwistAngularSoftLimit :datatype-) "kortex_driver/TwistAngularSoftLimit")
(setf (get kortex_driver::TwistAngularSoftLimit :definition-)
      "
uint32 control_mode
float32 twist_angular_soft_limit
")



(provide :kortex_driver/TwistAngularSoftLimit "ec20d4ed4983e99d6172917731a2eee5")


