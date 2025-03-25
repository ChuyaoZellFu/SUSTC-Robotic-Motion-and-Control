;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetMeasuredGripperMovement)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetMeasuredGripperMovement (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDGRIPPERMOVEMENT")
  (make-package "KORTEX_DRIVER::GETMEASUREDGRIPPERMOVEMENT"))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDGRIPPERMOVEMENTREQUEST")
  (make-package "KORTEX_DRIVER::GETMEASUREDGRIPPERMOVEMENTREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETMEASUREDGRIPPERMOVEMENTRESPONSE")
  (make-package "KORTEX_DRIVER::GETMEASUREDGRIPPERMOVEMENTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetMeasuredGripperMovementRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetMeasuredGripperMovementRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::GripperRequest :init))
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
    ;; kortex_driver/GripperRequest _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperRequest _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperRequest _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetMeasuredGripperMovementResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetMeasuredGripperMovementResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Gripper :init))
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
    ;; kortex_driver/Gripper _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Gripper _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Gripper _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetMeasuredGripperMovement
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetMeasuredGripperMovement :md5sum-) "8a811dcc01240e06c24ef1514e8bc606")
(setf (get kortex_driver::GetMeasuredGripperMovement :datatype-) "kortex_driver/GetMeasuredGripperMovement")
(setf (get kortex_driver::GetMeasuredGripperMovement :request) kortex_driver::GetMeasuredGripperMovementRequest)
(setf (get kortex_driver::GetMeasuredGripperMovement :response) kortex_driver::GetMeasuredGripperMovementResponse)

(defmethod kortex_driver::GetMeasuredGripperMovementRequest
  (:response () (instance kortex_driver::GetMeasuredGripperMovementResponse :init)))

(setf (get kortex_driver::GetMeasuredGripperMovementRequest :md5sum-) "8a811dcc01240e06c24ef1514e8bc606")
(setf (get kortex_driver::GetMeasuredGripperMovementRequest :datatype-) "kortex_driver/GetMeasuredGripperMovementRequest")
(setf (get kortex_driver::GetMeasuredGripperMovementRequest :definition-)
      "GripperRequest input

================================================================================
MSG: kortex_driver/GripperRequest

---
Gripper output

================================================================================
MSG: kortex_driver/Gripper

Finger[] finger
================================================================================
MSG: kortex_driver/Finger

uint32 finger_identifier
")

(setf (get kortex_driver::GetMeasuredGripperMovementResponse :md5sum-) "8a811dcc01240e06c24ef1514e8bc606")
(setf (get kortex_driver::GetMeasuredGripperMovementResponse :datatype-) "kortex_driver/GetMeasuredGripperMovementResponse")
(setf (get kortex_driver::GetMeasuredGripperMovementResponse :definition-)
      "GripperRequest input

================================================================================
MSG: kortex_driver/GripperRequest

---
Gripper output

================================================================================
MSG: kortex_driver/Gripper

Finger[] finger
================================================================================
MSG: kortex_driver/Finger

uint32 finger_identifier
")



(provide :kortex_driver/GetMeasuredGripperMovement "8a811dcc01240e06c24ef1514e8bc606")


