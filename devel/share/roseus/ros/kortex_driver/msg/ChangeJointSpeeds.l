;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ChangeJointSpeeds)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ChangeJointSpeeds (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CHANGEJOINTSPEEDS")
  (make-package "KORTEX_DRIVER::CHANGEJOINTSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude ChangeJointSpeeds.msg.html


(defclass kortex_driver::ChangeJointSpeeds
  :super ros::object
  :slots (_joint_speeds ))

(defmethod kortex_driver::ChangeJointSpeeds
  (:init
   (&key
    ((:joint_speeds __joint_speeds) (instance kortex_driver::Base_JointSpeeds :init))
    )
   (send-super :init)
   (setq _joint_speeds __joint_speeds)
   self)
  (:joint_speeds
   (&rest __joint_speeds)
   (if (keywordp (car __joint_speeds))
       (send* _joint_speeds __joint_speeds)
     (progn
       (if __joint_speeds (setq _joint_speeds (car __joint_speeds)))
       _joint_speeds)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Base_JointSpeeds _joint_speeds
    (send _joint_speeds :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Base_JointSpeeds _joint_speeds
       (send _joint_speeds :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Base_JointSpeeds _joint_speeds
     (send _joint_speeds :deserialize buf ptr-) (incf ptr- (send _joint_speeds :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ChangeJointSpeeds :md5sum-) "d6e510b2965f87e14832d5332810f679")
(setf (get kortex_driver::ChangeJointSpeeds :datatype-) "kortex_driver/ChangeJointSpeeds")
(setf (get kortex_driver::ChangeJointSpeeds :definition-)
      "
Base_JointSpeeds joint_speeds
================================================================================
MSG: kortex_driver/Base_JointSpeeds

JointSpeed[] joint_speeds
uint32 duration
================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
uint32 duration
")



(provide :kortex_driver/ChangeJointSpeeds "d6e510b2965f87e14832d5332810f679")


