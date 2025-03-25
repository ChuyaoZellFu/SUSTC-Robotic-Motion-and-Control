;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StopAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StopAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STOPACTION")
  (make-package "KORTEX_DRIVER::STOPACTION"))
(unless (find-package "KORTEX_DRIVER::STOPACTIONREQUEST")
  (make-package "KORTEX_DRIVER::STOPACTIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::STOPACTIONRESPONSE")
  (make-package "KORTEX_DRIVER::STOPACTIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::StopActionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::StopActionRequest
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

(defclass kortex_driver::StopActionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::StopActionResponse
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

(defclass kortex_driver::StopAction
  :super ros::object
  :slots ())

(setf (get kortex_driver::StopAction :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopAction :datatype-) "kortex_driver/StopAction")
(setf (get kortex_driver::StopAction :request) kortex_driver::StopActionRequest)
(setf (get kortex_driver::StopAction :response) kortex_driver::StopActionResponse)

(defmethod kortex_driver::StopActionRequest
  (:response () (instance kortex_driver::StopActionResponse :init)))

(setf (get kortex_driver::StopActionRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopActionRequest :datatype-) "kortex_driver/StopActionRequest")
(setf (get kortex_driver::StopActionRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::StopActionResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::StopActionResponse :datatype-) "kortex_driver/StopActionResponse")
(setf (get kortex_driver::StopActionResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/StopAction "f335b819dc59099fe3124b36f140ad07")


