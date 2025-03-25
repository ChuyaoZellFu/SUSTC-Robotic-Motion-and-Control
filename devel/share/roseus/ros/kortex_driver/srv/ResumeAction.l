;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResumeAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResumeAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESUMEACTION")
  (make-package "KORTEX_DRIVER::RESUMEACTION"))
(unless (find-package "KORTEX_DRIVER::RESUMEACTIONREQUEST")
  (make-package "KORTEX_DRIVER::RESUMEACTIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESUMEACTIONRESPONSE")
  (make-package "KORTEX_DRIVER::RESUMEACTIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResumeActionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResumeActionRequest
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

(defclass kortex_driver::ResumeActionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResumeActionResponse
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

(defclass kortex_driver::ResumeAction
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResumeAction :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResumeAction :datatype-) "kortex_driver/ResumeAction")
(setf (get kortex_driver::ResumeAction :request) kortex_driver::ResumeActionRequest)
(setf (get kortex_driver::ResumeAction :response) kortex_driver::ResumeActionResponse)

(defmethod kortex_driver::ResumeActionRequest
  (:response () (instance kortex_driver::ResumeActionResponse :init)))

(setf (get kortex_driver::ResumeActionRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResumeActionRequest :datatype-) "kortex_driver/ResumeActionRequest")
(setf (get kortex_driver::ResumeActionRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ResumeActionResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ResumeActionResponse :datatype-) "kortex_driver/ResumeActionResponse")
(setf (get kortex_driver::ResumeActionResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ResumeAction "f335b819dc59099fe3124b36f140ad07")


