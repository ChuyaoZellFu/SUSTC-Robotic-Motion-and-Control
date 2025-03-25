;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PauseAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PauseAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PAUSEACTION")
  (make-package "KORTEX_DRIVER::PAUSEACTION"))
(unless (find-package "KORTEX_DRIVER::PAUSEACTIONREQUEST")
  (make-package "KORTEX_DRIVER::PAUSEACTIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::PAUSEACTIONRESPONSE")
  (make-package "KORTEX_DRIVER::PAUSEACTIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PauseActionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PauseActionRequest
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

(defclass kortex_driver::PauseActionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PauseActionResponse
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

(defclass kortex_driver::PauseAction
  :super ros::object
  :slots ())

(setf (get kortex_driver::PauseAction :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::PauseAction :datatype-) "kortex_driver/PauseAction")
(setf (get kortex_driver::PauseAction :request) kortex_driver::PauseActionRequest)
(setf (get kortex_driver::PauseAction :response) kortex_driver::PauseActionResponse)

(defmethod kortex_driver::PauseActionRequest
  (:response () (instance kortex_driver::PauseActionResponse :init)))

(setf (get kortex_driver::PauseActionRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::PauseActionRequest :datatype-) "kortex_driver/PauseActionRequest")
(setf (get kortex_driver::PauseActionRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PauseActionResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::PauseActionResponse :datatype-) "kortex_driver/PauseActionResponse")
(setf (get kortex_driver::PauseActionResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PauseAction "f335b819dc59099fe3124b36f140ad07")


