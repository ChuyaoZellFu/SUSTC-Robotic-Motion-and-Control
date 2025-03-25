;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::IKData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'IKData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::IKDATA")
  (make-package "KORTEX_DRIVER::IKDATA"))

(in-package "ROS")
;;//! \htmlinclude IKData.msg.html


(defclass kortex_driver::IKData
  :super ros::object
  :slots (_cartesian_pose _guess ))

(defmethod kortex_driver::IKData
  (:init
   (&key
    ((:cartesian_pose __cartesian_pose) (instance kortex_driver::Pose :init))
    ((:guess __guess) (instance kortex_driver::JointAngles :init))
    )
   (send-super :init)
   (setq _cartesian_pose __cartesian_pose)
   (setq _guess __guess)
   self)
  (:cartesian_pose
   (&rest __cartesian_pose)
   (if (keywordp (car __cartesian_pose))
       (send* _cartesian_pose __cartesian_pose)
     (progn
       (if __cartesian_pose (setq _cartesian_pose (car __cartesian_pose)))
       _cartesian_pose)))
  (:guess
   (&rest __guess)
   (if (keywordp (car __guess))
       (send* _guess __guess)
     (progn
       (if __guess (setq _guess (car __guess)))
       _guess)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Pose _cartesian_pose
    (send _cartesian_pose :serialization-length)
    ;; kortex_driver/JointAngles _guess
    (send _guess :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Pose _cartesian_pose
       (send _cartesian_pose :serialize s)
     ;; kortex_driver/JointAngles _guess
       (send _guess :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Pose _cartesian_pose
     (send _cartesian_pose :deserialize buf ptr-) (incf ptr- (send _cartesian_pose :serialization-length))
   ;; kortex_driver/JointAngles _guess
     (send _guess :deserialize buf ptr-) (incf ptr- (send _guess :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::IKData :md5sum-) "29f05c9210572828af7df145fee29d3b")
(setf (get kortex_driver::IKData :datatype-) "kortex_driver/IKData")
(setf (get kortex_driver::IKData :definition-)
      "
Pose cartesian_pose
JointAngles guess
================================================================================
MSG: kortex_driver/Pose

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
float32 value
")



(provide :kortex_driver/IKData "29f05c9210572828af7df145fee29d3b")


