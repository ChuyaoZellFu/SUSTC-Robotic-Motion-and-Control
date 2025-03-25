;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointTorques)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointTorques (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTTORQUES")
  (make-package "KORTEX_DRIVER::JOINTTORQUES"))

(in-package "ROS")
;;//! \htmlinclude JointTorques.msg.html


(defclass kortex_driver::JointTorques
  :super ros::object
  :slots (_joint_torques _duration ))

(defmethod kortex_driver::JointTorques
  (:init
   (&key
    ((:joint_torques __joint_torques) ())
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _joint_torques __joint_torques)
   (setq _duration (round __duration))
   self)
  (:joint_torques
   (&rest __joint_torques)
   (if (keywordp (car __joint_torques))
       (send* _joint_torques __joint_torques)
     (progn
       (if __joint_torques (setq _joint_torques (car __joint_torques)))
       _joint_torques)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/JointTorque[] _joint_torques
    (apply #'+ (send-all _joint_torques :serialization-length)) 4
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointTorque[] _joint_torques
     (write-long (length _joint_torques) s)
     (dolist (elem _joint_torques)
       (send elem :serialize s)
       )
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointTorque[] _joint_torques
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_torques (let (r) (dotimes (i n) (push (instance kortex_driver::JointTorque :init) r)) r))
     (dolist (elem- _joint_torques)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::JointTorques :md5sum-) "ada0e26af44ca932f53e6c0e450fadd8")
(setf (get kortex_driver::JointTorques :datatype-) "kortex_driver/JointTorques")
(setf (get kortex_driver::JointTorques :definition-)
      "
JointTorque[] joint_torques
uint32 duration
================================================================================
MSG: kortex_driver/JointTorque

uint32 joint_identifier
float32 value
uint32 duration
")



(provide :kortex_driver/JointTorques "ada0e26af44ca932f53e6c0e450fadd8")


