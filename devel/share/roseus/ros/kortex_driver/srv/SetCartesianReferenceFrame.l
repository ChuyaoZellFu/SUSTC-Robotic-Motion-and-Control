;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetCartesianReferenceFrame)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetCartesianReferenceFrame (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCARTESIANREFERENCEFRAME")
  (make-package "KORTEX_DRIVER::SETCARTESIANREFERENCEFRAME"))
(unless (find-package "KORTEX_DRIVER::SETCARTESIANREFERENCEFRAMEREQUEST")
  (make-package "KORTEX_DRIVER::SETCARTESIANREFERENCEFRAMEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCARTESIANREFERENCEFRAMERESPONSE")
  (make-package "KORTEX_DRIVER::SETCARTESIANREFERENCEFRAMERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetCartesianReferenceFrameRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetCartesianReferenceFrameRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::CartesianReferenceFrameInfo :init))
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
    ;; kortex_driver/CartesianReferenceFrameInfo _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CartesianReferenceFrameInfo _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CartesianReferenceFrameInfo _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetCartesianReferenceFrameResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetCartesianReferenceFrameResponse
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

(defclass kortex_driver::SetCartesianReferenceFrame
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetCartesianReferenceFrame :md5sum-) "80417b91d14dfe7b5a3f569e00b74176")
(setf (get kortex_driver::SetCartesianReferenceFrame :datatype-) "kortex_driver/SetCartesianReferenceFrame")
(setf (get kortex_driver::SetCartesianReferenceFrame :request) kortex_driver::SetCartesianReferenceFrameRequest)
(setf (get kortex_driver::SetCartesianReferenceFrame :response) kortex_driver::SetCartesianReferenceFrameResponse)

(defmethod kortex_driver::SetCartesianReferenceFrameRequest
  (:response () (instance kortex_driver::SetCartesianReferenceFrameResponse :init)))

(setf (get kortex_driver::SetCartesianReferenceFrameRequest :md5sum-) "80417b91d14dfe7b5a3f569e00b74176")
(setf (get kortex_driver::SetCartesianReferenceFrameRequest :datatype-) "kortex_driver/SetCartesianReferenceFrameRequest")
(setf (get kortex_driver::SetCartesianReferenceFrameRequest :definition-)
      "CartesianReferenceFrameInfo input

================================================================================
MSG: kortex_driver/CartesianReferenceFrameInfo

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetCartesianReferenceFrameResponse :md5sum-) "80417b91d14dfe7b5a3f569e00b74176")
(setf (get kortex_driver::SetCartesianReferenceFrameResponse :datatype-) "kortex_driver/SetCartesianReferenceFrameResponse")
(setf (get kortex_driver::SetCartesianReferenceFrameResponse :definition-)
      "CartesianReferenceFrameInfo input

================================================================================
MSG: kortex_driver/CartesianReferenceFrameInfo

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetCartesianReferenceFrame "80417b91d14dfe7b5a3f569e00b74176")


