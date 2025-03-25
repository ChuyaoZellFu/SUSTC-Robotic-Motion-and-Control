;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Base_JointSpeeds)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Base_JointSpeeds (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASE_JOINTSPEEDS")
  (make-package "KORTEX_DRIVER::BASE_JOINTSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude Base_JointSpeeds.msg.html


(defclass kortex_driver::Base_JointSpeeds
  :super ros::object
  :slots (_joint_speeds _duration ))

(defmethod kortex_driver::Base_JointSpeeds
  (:init
   (&key
    ((:joint_speeds __joint_speeds) ())
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _joint_speeds __joint_speeds)
   (setq _duration (round __duration))
   self)
  (:joint_speeds
   (&rest __joint_speeds)
   (if (keywordp (car __joint_speeds))
       (send* _joint_speeds __joint_speeds)
     (progn
       (if __joint_speeds (setq _joint_speeds (car __joint_speeds)))
       _joint_speeds)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/JointSpeed[] _joint_speeds
    (apply #'+ (send-all _joint_speeds :serialization-length)) 4
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointSpeed[] _joint_speeds
     (write-long (length _joint_speeds) s)
     (dolist (elem _joint_speeds)
       (send elem :serialize s)
       )
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointSpeed[] _joint_speeds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_speeds (let (r) (dotimes (i n) (push (instance kortex_driver::JointSpeed :init) r)) r))
     (dolist (elem- _joint_speeds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Base_JointSpeeds :md5sum-) "f323c64d027d947f420c021696580eb4")
(setf (get kortex_driver::Base_JointSpeeds :datatype-) "kortex_driver/Base_JointSpeeds")
(setf (get kortex_driver::Base_JointSpeeds :definition-)
      "
JointSpeed[] joint_speeds
uint32 duration
================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
uint32 duration
")



(provide :kortex_driver/Base_JointSpeeds "f323c64d027d947f420c021696580eb4")


