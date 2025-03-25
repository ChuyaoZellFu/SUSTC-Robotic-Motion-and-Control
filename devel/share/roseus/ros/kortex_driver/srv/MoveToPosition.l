;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MoveToPosition)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MoveToPosition (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MOVETOPOSITION")
  (make-package "KORTEX_DRIVER::MOVETOPOSITION"))
(unless (find-package "KORTEX_DRIVER::MOVETOPOSITIONREQUEST")
  (make-package "KORTEX_DRIVER::MOVETOPOSITIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::MOVETOPOSITIONRESPONSE")
  (make-package "KORTEX_DRIVER::MOVETOPOSITIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::MoveToPositionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::MoveToPositionRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::PositionCommand :init))
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
    ;; kortex_driver/PositionCommand _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/PositionCommand _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/PositionCommand _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::MoveToPositionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::MoveToPositionResponse
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

(defclass kortex_driver::MoveToPosition
  :super ros::object
  :slots ())

(setf (get kortex_driver::MoveToPosition :md5sum-) "e232d9e676cf9963716162da0597fb96")
(setf (get kortex_driver::MoveToPosition :datatype-) "kortex_driver/MoveToPosition")
(setf (get kortex_driver::MoveToPosition :request) kortex_driver::MoveToPositionRequest)
(setf (get kortex_driver::MoveToPosition :response) kortex_driver::MoveToPositionResponse)

(defmethod kortex_driver::MoveToPositionRequest
  (:response () (instance kortex_driver::MoveToPositionResponse :init)))

(setf (get kortex_driver::MoveToPositionRequest :md5sum-) "e232d9e676cf9963716162da0597fb96")
(setf (get kortex_driver::MoveToPositionRequest :datatype-) "kortex_driver/MoveToPositionRequest")
(setf (get kortex_driver::MoveToPositionRequest :definition-)
      "PositionCommand input

================================================================================
MSG: kortex_driver/PositionCommand

float32 position
float32 velocity
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::MoveToPositionResponse :md5sum-) "e232d9e676cf9963716162da0597fb96")
(setf (get kortex_driver::MoveToPositionResponse :datatype-) "kortex_driver/MoveToPositionResponse")
(setf (get kortex_driver::MoveToPositionResponse :definition-)
      "PositionCommand input

================================================================================
MSG: kortex_driver/PositionCommand

float32 position
float32 velocity
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/MoveToPosition "e232d9e676cf9963716162da0597fb96")


