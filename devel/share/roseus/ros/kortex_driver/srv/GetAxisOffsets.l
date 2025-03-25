;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetAxisOffsets)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetAxisOffsets (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETAXISOFFSETS")
  (make-package "KORTEX_DRIVER::GETAXISOFFSETS"))
(unless (find-package "KORTEX_DRIVER::GETAXISOFFSETSREQUEST")
  (make-package "KORTEX_DRIVER::GETAXISOFFSETSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETAXISOFFSETSRESPONSE")
  (make-package "KORTEX_DRIVER::GETAXISOFFSETSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetAxisOffsetsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetAxisOffsetsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAxisOffsetsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetAxisOffsetsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::AxisOffsets :init))
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
    ;; kortex_driver/AxisOffsets _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/AxisOffsets _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/AxisOffsets _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetAxisOffsets
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetAxisOffsets :md5sum-) "e0bbfa31b1895cb9c1c9ec4bbac51a98")
(setf (get kortex_driver::GetAxisOffsets :datatype-) "kortex_driver/GetAxisOffsets")
(setf (get kortex_driver::GetAxisOffsets :request) kortex_driver::GetAxisOffsetsRequest)
(setf (get kortex_driver::GetAxisOffsets :response) kortex_driver::GetAxisOffsetsResponse)

(defmethod kortex_driver::GetAxisOffsetsRequest
  (:response () (instance kortex_driver::GetAxisOffsetsResponse :init)))

(setf (get kortex_driver::GetAxisOffsetsRequest :md5sum-) "e0bbfa31b1895cb9c1c9ec4bbac51a98")
(setf (get kortex_driver::GetAxisOffsetsRequest :datatype-) "kortex_driver/GetAxisOffsetsRequest")
(setf (get kortex_driver::GetAxisOffsetsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
AxisOffsets output

================================================================================
MSG: kortex_driver/AxisOffsets

float32 absolute_offset
")

(setf (get kortex_driver::GetAxisOffsetsResponse :md5sum-) "e0bbfa31b1895cb9c1c9ec4bbac51a98")
(setf (get kortex_driver::GetAxisOffsetsResponse :datatype-) "kortex_driver/GetAxisOffsetsResponse")
(setf (get kortex_driver::GetAxisOffsetsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
AxisOffsets output

================================================================================
MSG: kortex_driver/AxisOffsets

float32 absolute_offset
")



(provide :kortex_driver/GetAxisOffsets "e0bbfa31b1895cb9c1c9ec4bbac51a98")


