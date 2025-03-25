;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetCartesianReferenceFrame)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetCartesianReferenceFrame (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCARTESIANREFERENCEFRAME")
  (make-package "KORTEX_DRIVER::GETCARTESIANREFERENCEFRAME"))
(unless (find-package "KORTEX_DRIVER::GETCARTESIANREFERENCEFRAMEREQUEST")
  (make-package "KORTEX_DRIVER::GETCARTESIANREFERENCEFRAMEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCARTESIANREFERENCEFRAMERESPONSE")
  (make-package "KORTEX_DRIVER::GETCARTESIANREFERENCEFRAMERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetCartesianReferenceFrameRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetCartesianReferenceFrameRequest
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

(defclass kortex_driver::GetCartesianReferenceFrameResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetCartesianReferenceFrameResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CartesianReferenceFrameInfo :init))
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
    ;; kortex_driver/CartesianReferenceFrameInfo _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CartesianReferenceFrameInfo _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CartesianReferenceFrameInfo _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetCartesianReferenceFrame
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetCartesianReferenceFrame :md5sum-) "fd1f94f7574ce13875b4bba8e911f051")
(setf (get kortex_driver::GetCartesianReferenceFrame :datatype-) "kortex_driver/GetCartesianReferenceFrame")
(setf (get kortex_driver::GetCartesianReferenceFrame :request) kortex_driver::GetCartesianReferenceFrameRequest)
(setf (get kortex_driver::GetCartesianReferenceFrame :response) kortex_driver::GetCartesianReferenceFrameResponse)

(defmethod kortex_driver::GetCartesianReferenceFrameRequest
  (:response () (instance kortex_driver::GetCartesianReferenceFrameResponse :init)))

(setf (get kortex_driver::GetCartesianReferenceFrameRequest :md5sum-) "fd1f94f7574ce13875b4bba8e911f051")
(setf (get kortex_driver::GetCartesianReferenceFrameRequest :datatype-) "kortex_driver/GetCartesianReferenceFrameRequest")
(setf (get kortex_driver::GetCartesianReferenceFrameRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CartesianReferenceFrameInfo output

================================================================================
MSG: kortex_driver/CartesianReferenceFrameInfo

")

(setf (get kortex_driver::GetCartesianReferenceFrameResponse :md5sum-) "fd1f94f7574ce13875b4bba8e911f051")
(setf (get kortex_driver::GetCartesianReferenceFrameResponse :datatype-) "kortex_driver/GetCartesianReferenceFrameResponse")
(setf (get kortex_driver::GetCartesianReferenceFrameResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CartesianReferenceFrameInfo output

================================================================================
MSG: kortex_driver/CartesianReferenceFrameInfo

")



(provide :kortex_driver/GetCartesianReferenceFrame "fd1f94f7574ce13875b4bba8e911f051")


