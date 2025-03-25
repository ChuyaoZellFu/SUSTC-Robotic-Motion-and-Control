;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TrajectoryInfo)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TrajectoryInfo (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRAJECTORYINFO")
  (make-package "KORTEX_DRIVER::TRAJECTORYINFO"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryInfo.msg.html


(defclass kortex_driver::TrajectoryInfo
  :super ros::object
  :slots (_trajectory_info_type _waypoint_index _joint_index ))

(defmethod kortex_driver::TrajectoryInfo
  (:init
   (&key
    ((:trajectory_info_type __trajectory_info_type) 0)
    ((:waypoint_index __waypoint_index) 0)
    ((:joint_index __joint_index) 0)
    )
   (send-super :init)
   (setq _trajectory_info_type (round __trajectory_info_type))
   (setq _waypoint_index (round __waypoint_index))
   (setq _joint_index (round __joint_index))
   self)
  (:trajectory_info_type
   (&optional __trajectory_info_type)
   (if __trajectory_info_type (setq _trajectory_info_type __trajectory_info_type)) _trajectory_info_type)
  (:waypoint_index
   (&optional __waypoint_index)
   (if __waypoint_index (setq _waypoint_index __waypoint_index)) _waypoint_index)
  (:joint_index
   (&optional __joint_index)
   (if __joint_index (setq _joint_index __joint_index)) _joint_index)
  (:serialization-length
   ()
   (+
    ;; uint32 _trajectory_info_type
    4
    ;; uint32 _waypoint_index
    4
    ;; uint32 _joint_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _trajectory_info_type
       (write-long _trajectory_info_type s)
     ;; uint32 _waypoint_index
       (write-long _waypoint_index s)
     ;; uint32 _joint_index
       (write-long _joint_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _trajectory_info_type
     (setq _trajectory_info_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _waypoint_index
     (setq _waypoint_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _joint_index
     (setq _joint_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TrajectoryInfo :md5sum-) "0eff35f5790d1aa2c620bfca263340d6")
(setf (get kortex_driver::TrajectoryInfo :datatype-) "kortex_driver/TrajectoryInfo")
(setf (get kortex_driver::TrajectoryInfo :definition-)
      "
uint32 trajectory_info_type
uint32 waypoint_index
uint32 joint_index
")



(provide :kortex_driver/TrajectoryInfo "0eff35f5790d1aa2c620bfca263340d6")


