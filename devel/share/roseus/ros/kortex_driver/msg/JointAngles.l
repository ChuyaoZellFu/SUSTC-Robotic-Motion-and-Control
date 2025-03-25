;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::JointAngles)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'JointAngles (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::JOINTANGLES")
  (make-package "KORTEX_DRIVER::JOINTANGLES"))

(in-package "ROS")
;;//! \htmlinclude JointAngles.msg.html


(defclass kortex_driver::JointAngles
  :super ros::object
  :slots (_joint_angles ))

(defmethod kortex_driver::JointAngles
  (:init
   (&key
    ((:joint_angles __joint_angles) ())
    )
   (send-super :init)
   (setq _joint_angles __joint_angles)
   self)
  (:joint_angles
   (&rest __joint_angles)
   (if (keywordp (car __joint_angles))
       (send* _joint_angles __joint_angles)
     (progn
       (if __joint_angles (setq _joint_angles (car __joint_angles)))
       _joint_angles)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/JointAngle[] _joint_angles
    (apply #'+ (send-all _joint_angles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointAngle[] _joint_angles
     (write-long (length _joint_angles) s)
     (dolist (elem _joint_angles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointAngle[] _joint_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles (let (r) (dotimes (i n) (push (instance kortex_driver::JointAngle :init) r)) r))
     (dolist (elem- _joint_angles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::JointAngles :md5sum-) "94886d22261db2d621b5fe4c4bffdfa1")
(setf (get kortex_driver::JointAngles :datatype-) "kortex_driver/JointAngles")
(setf (get kortex_driver::JointAngles :definition-)
      "
JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
float32 value
")



(provide :kortex_driver/JointAngles "94886d22261db2d621b5fe4c4bffdfa1")


