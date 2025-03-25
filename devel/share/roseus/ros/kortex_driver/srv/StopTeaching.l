;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StopTeaching)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StopTeaching (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STOPTEACHING")
  (make-package "KORTEX_DRIVER::STOPTEACHING"))
(unless (find-package "KORTEX_DRIVER::STOPTEACHINGREQUEST")
  (make-package "KORTEX_DRIVER::STOPTEACHINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::STOPTEACHINGRESPONSE")
  (make-package "KORTEX_DRIVER::STOPTEACHINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::StopTeachingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::StopTeachingRequest
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

(defclass kortex_driver::StopTeachingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::StopTeachingResponse
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

(defclass kortex_driver::StopTeaching
  :super ros::object
  :slots ())

(setf (get kortex_driver::StopTeaching :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopTeaching :datatype-) "kortex_driver/StopTeaching")
(setf (get kortex_driver::StopTeaching :request) kortex_driver::StopTeachingRequest)
(setf (get kortex_driver::StopTeaching :response) kortex_driver::StopTeachingResponse)

(defmethod kortex_driver::StopTeachingRequest
  (:response () (instance kortex_driver::StopTeachingResponse :init)))

(setf (get kortex_driver::StopTeachingRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopTeachingRequest :datatype-) "kortex_driver/StopTeachingRequest")
(setf (get kortex_driver::StopTeachingRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::StopTeachingResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopTeachingResponse :datatype-) "kortex_driver/StopTeachingResponse")
(setf (get kortex_driver::StopTeachingResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/StopTeaching "f335b819dc59099fe3124b36f140ad07")


