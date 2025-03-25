;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetPartNumberRevision)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetPartNumberRevision (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETPARTNUMBERREVISION")
  (make-package "KORTEX_DRIVER::GETPARTNUMBERREVISION"))
(unless (find-package "KORTEX_DRIVER::GETPARTNUMBERREVISIONREQUEST")
  (make-package "KORTEX_DRIVER::GETPARTNUMBERREVISIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETPARTNUMBERREVISIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETPARTNUMBERREVISIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetPartNumberRevisionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetPartNumberRevisionRequest
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

(defclass kortex_driver::GetPartNumberRevisionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetPartNumberRevisionResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::PartNumberRevision :init))
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
    ;; kortex_driver/PartNumberRevision _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/PartNumberRevision _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/PartNumberRevision _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetPartNumberRevision
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetPartNumberRevision :md5sum-) "a25670b2402eb62859a04b7bd6a88b8c")
(setf (get kortex_driver::GetPartNumberRevision :datatype-) "kortex_driver/GetPartNumberRevision")
(setf (get kortex_driver::GetPartNumberRevision :request) kortex_driver::GetPartNumberRevisionRequest)
(setf (get kortex_driver::GetPartNumberRevision :response) kortex_driver::GetPartNumberRevisionResponse)

(defmethod kortex_driver::GetPartNumberRevisionRequest
  (:response () (instance kortex_driver::GetPartNumberRevisionResponse :init)))

(setf (get kortex_driver::GetPartNumberRevisionRequest :md5sum-) "a25670b2402eb62859a04b7bd6a88b8c")
(setf (get kortex_driver::GetPartNumberRevisionRequest :datatype-) "kortex_driver/GetPartNumberRevisionRequest")
(setf (get kortex_driver::GetPartNumberRevisionRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
PartNumberRevision output

================================================================================
MSG: kortex_driver/PartNumberRevision

")

(setf (get kortex_driver::GetPartNumberRevisionResponse :md5sum-) "a25670b2402eb62859a04b7bd6a88b8c")
(setf (get kortex_driver::GetPartNumberRevisionResponse :datatype-) "kortex_driver/GetPartNumberRevisionResponse")
(setf (get kortex_driver::GetPartNumberRevisionResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
PartNumberRevision output

================================================================================
MSG: kortex_driver/PartNumberRevision

")



(provide :kortex_driver/GetPartNumberRevision "a25670b2402eb62859a04b7bd6a88b8c")


