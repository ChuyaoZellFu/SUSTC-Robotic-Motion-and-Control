;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetDesiredJointSpeeds)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetDesiredJointSpeeds (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETDESIREDJOINTSPEEDS")
  (make-package "KORTEX_DRIVER::SETDESIREDJOINTSPEEDS"))
(unless (find-package "KORTEX_DRIVER::SETDESIREDJOINTSPEEDSREQUEST")
  (make-package "KORTEX_DRIVER::SETDESIREDJOINTSPEEDSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETDESIREDJOINTSPEEDSRESPONSE")
  (make-package "KORTEX_DRIVER::SETDESIREDJOINTSPEEDSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetDesiredJointSpeedsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetDesiredJointSpeedsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControlConfig_JointSpeeds :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControlConfig_JointSpeeds _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControlConfig_JointSpeeds _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControlConfig_JointSpeeds _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetDesiredJointSpeedsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetDesiredJointSpeedsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetDesiredJointSpeeds
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetDesiredJointSpeeds :md5sum-) "57cd70ec5519968c6e959068a3575705")
(setf (get kortex_driver::SetDesiredJointSpeeds :datatype-) "kortex_driver/SetDesiredJointSpeeds")
(setf (get kortex_driver::SetDesiredJointSpeeds :request) kortex_driver::SetDesiredJointSpeedsRequest)
(setf (get kortex_driver::SetDesiredJointSpeeds :response) kortex_driver::SetDesiredJointSpeedsResponse)

(defmethod kortex_driver::SetDesiredJointSpeedsRequest
  (:response () (instance kortex_driver::SetDesiredJointSpeedsResponse :init)))

(setf (get kortex_driver::SetDesiredJointSpeedsRequest :md5sum-) "57cd70ec5519968c6e959068a3575705")
(setf (get kortex_driver::SetDesiredJointSpeedsRequest :datatype-) "kortex_driver/SetDesiredJointSpeedsRequest")
(setf (get kortex_driver::SetDesiredJointSpeedsRequest :definition-)
      "ControlConfig_JointSpeeds input

================================================================================
MSG: kortex_driver/ControlConfig_JointSpeeds

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetDesiredJointSpeedsResponse :md5sum-) "57cd70ec5519968c6e959068a3575705")
(setf (get kortex_driver::SetDesiredJointSpeedsResponse :datatype-) "kortex_driver/SetDesiredJointSpeedsResponse")
(setf (get kortex_driver::SetDesiredJointSpeedsResponse :definition-)
      "ControlConfig_JointSpeeds input

================================================================================
MSG: kortex_driver/ControlConfig_JointSpeeds

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetDesiredJointSpeeds "57cd70ec5519968c6e959068a3575705")


