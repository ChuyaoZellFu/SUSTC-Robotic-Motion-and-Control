;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlayCartesianTrajectoryPosition)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlayCartesianTrajectoryPosition (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYPOSITION")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYPOSITION"))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYPOSITIONREQUEST")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYPOSITIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYPOSITIONRESPONSE")
  (make-package "KORTEX_DRIVER::PLAYCARTESIANTRAJECTORYPOSITIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlayCartesianTrajectoryPositionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlayCartesianTrajectoryPositionRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ConstrainedPosition :init))
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
    ;; kortex_driver/ConstrainedPosition _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ConstrainedPosition _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ConstrainedPosition _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlayCartesianTrajectoryPositionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlayCartesianTrajectoryPositionResponse
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

(defclass kortex_driver::PlayCartesianTrajectoryPosition
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlayCartesianTrajectoryPosition :md5sum-) "da1eaa51dfbdd2438263c790b42891de")
(setf (get kortex_driver::PlayCartesianTrajectoryPosition :datatype-) "kortex_driver/PlayCartesianTrajectoryPosition")
(setf (get kortex_driver::PlayCartesianTrajectoryPosition :request) kortex_driver::PlayCartesianTrajectoryPositionRequest)
(setf (get kortex_driver::PlayCartesianTrajectoryPosition :response) kortex_driver::PlayCartesianTrajectoryPositionResponse)

(defmethod kortex_driver::PlayCartesianTrajectoryPositionRequest
  (:response () (instance kortex_driver::PlayCartesianTrajectoryPositionResponse :init)))

(setf (get kortex_driver::PlayCartesianTrajectoryPositionRequest :md5sum-) "da1eaa51dfbdd2438263c790b42891de")
(setf (get kortex_driver::PlayCartesianTrajectoryPositionRequest :datatype-) "kortex_driver/PlayCartesianTrajectoryPositionRequest")
(setf (get kortex_driver::PlayCartesianTrajectoryPositionRequest :definition-)
      "ConstrainedPosition input

================================================================================
MSG: kortex_driver/ConstrainedPosition

Base_Position target_position
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Base_Position

float32 x
float32 y
float32 z
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

(setf (get kortex_driver::PlayCartesianTrajectoryPositionResponse :md5sum-) "da1eaa51dfbdd2438263c790b42891de")
(setf (get kortex_driver::PlayCartesianTrajectoryPositionResponse :datatype-) "kortex_driver/PlayCartesianTrajectoryPositionResponse")
(setf (get kortex_driver::PlayCartesianTrajectoryPositionResponse :definition-)
      "ConstrainedPosition input

================================================================================
MSG: kortex_driver/ConstrainedPosition

Base_Position target_position
CartesianTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/Base_Position

float32 x
float32 y
float32 z
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



(provide :kortex_driver/PlayCartesianTrajectoryPosition "da1eaa51dfbdd2438263c790b42891de")


