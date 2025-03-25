;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetAxisOffsets)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetAxisOffsets (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETAXISOFFSETS")
  (make-package "KORTEX_DRIVER::SETAXISOFFSETS"))
(unless (find-package "KORTEX_DRIVER::SETAXISOFFSETSREQUEST")
  (make-package "KORTEX_DRIVER::SETAXISOFFSETSREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETAXISOFFSETSRESPONSE")
  (make-package "KORTEX_DRIVER::SETAXISOFFSETSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetAxisOffsetsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetAxisOffsetsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::AxisPosition :init))
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
    ;; kortex_driver/AxisPosition _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/AxisPosition _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/AxisPosition _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetAxisOffsetsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetAxisOffsetsResponse
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

(defclass kortex_driver::SetAxisOffsets
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetAxisOffsets :md5sum-) "b3df8f041f59058a98542ed76394a7b4")
(setf (get kortex_driver::SetAxisOffsets :datatype-) "kortex_driver/SetAxisOffsets")
(setf (get kortex_driver::SetAxisOffsets :request) kortex_driver::SetAxisOffsetsRequest)
(setf (get kortex_driver::SetAxisOffsets :response) kortex_driver::SetAxisOffsetsResponse)

(defmethod kortex_driver::SetAxisOffsetsRequest
  (:response () (instance kortex_driver::SetAxisOffsetsResponse :init)))

(setf (get kortex_driver::SetAxisOffsetsRequest :md5sum-) "b3df8f041f59058a98542ed76394a7b4")
(setf (get kortex_driver::SetAxisOffsetsRequest :datatype-) "kortex_driver/SetAxisOffsetsRequest")
(setf (get kortex_driver::SetAxisOffsetsRequest :definition-)
      "AxisPosition input

================================================================================
MSG: kortex_driver/AxisPosition

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetAxisOffsetsResponse :md5sum-) "b3df8f041f59058a98542ed76394a7b4")
(setf (get kortex_driver::SetAxisOffsetsResponse :datatype-) "kortex_driver/SetAxisOffsetsResponse")
(setf (get kortex_driver::SetAxisOffsetsResponse :definition-)
      "AxisPosition input

================================================================================
MSG: kortex_driver/AxisPosition

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetAxisOffsets "b3df8f041f59058a98542ed76394a7b4")


