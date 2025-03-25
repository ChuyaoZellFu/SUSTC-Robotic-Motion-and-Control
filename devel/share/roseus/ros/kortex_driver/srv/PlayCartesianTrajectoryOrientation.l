;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlayCartesianTrajectoryOrientation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlayCartesianTrajectoryOrientation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYORIENTATION")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYORIENTATION"))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYORIENTATIONREQUEST")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYORIENTATIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYORIENTATIONRESPONSE")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYORIENTATIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlayCartesianTrajectoryOrientationRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlayCartesianTrajectoryOrientationRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ConstrainedOrientation :init))
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
    ;; kortex_driver/ConstrainedOrientation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ConstrainedOrientation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ConstrainedOrientation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlayCartesianTrajectoryOrientationResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlayCartesianTrajectoryOrientationResponse
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

(defclass kortex_driver::PlayCartesianTrajectoryOrientation
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlayCartesianTrajectoryOrientation :md5sum-) "118c3171acc8dd69436f0f28b7925e31")
(setf (get kortex_driver::PlayCartesianTrajectoryOrientation :datatype-) "kortex_driver/PlayCartesianTrajectoryOrientation")
(setf (get kortex_driver::PlayCartesianTrajectoryOrientation :request) kortex_driver::PlayCartesianTrajectoryOrientationRequest)
(setf (get kortex_driver::PlayCartesianTrajectoryOrientation :response) kortex_driver::PlayCartesianTrajectoryOrientationResponse)

(defmethod kortex_driver::PlayCartesianTrajectoryOrientationRequest
  (:response () (instance kortex_driver::PlayCartesianTrajectoryOrientationResponse :init)))

(setf (get kortex_driver::PlayCartesianTrajectoryOrientationRequest :md5sum-) "118c3171acc8dd69436f0f28b7925e31")
(setf (get kortex_driver::PlayCartesianTrajectoryOrientationRequest :datatype-) "kortex_driver/PlayCartesianTrajectoryOrientationRequest")
(setf (get kortex_driver::PlayCartesianTrajectoryOrientationRequest :definition-)
      "ConstrainedOrientation input

================================================================================
MSG: kortex_driver/ConstrainedOrientation

Orientation target_orientation
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Orientation

float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint

CartesianTrajectoryConstraint_type oneof_type
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint_type

CartesianSpeed[] speed
uint32[] duration
================================================================================
MSG: kortex_driver/CartesianSpeed

float32 translation
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PlayCartesianTrajectoryOrientationResponse :md5sum-) "118c3171acc8dd69436f0f28b7925e31")
(setf (get kortex_driver::PlayCartesianTrajectoryOrientationResponse :datatype-) "kortex_driver/PlayCartesianTrajectoryOrientationResponse")
(setf (get kortex_driver::PlayCartesianTrajectoryOrientationResponse :definition-)
      "ConstrainedOrientation input

================================================================================
MSG: kortex_driver/ConstrainedOrientation

Orientation target_orientation
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Orientation

float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint

CartesianTrajectoryConstraint_type oneof_type
================================================================================
MSG: kortex_driver/CartesianTrajectoryConstraint_type

CartesianSpeed[] speed
uint32[] duration
================================================================================
MSG: kortex_driver/CartesianSpeed

float32 translation
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PlayCartesianTrajectoryOrientation "118c3171acc8dd69436f0f28b7925e31")


