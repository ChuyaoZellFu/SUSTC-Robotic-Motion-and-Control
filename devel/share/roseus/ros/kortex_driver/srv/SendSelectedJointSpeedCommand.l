;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendSelectedJointSpeedCommand)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendSelectedJointSpeedCommand (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDCOMMAND")
  (make-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDCOMMAND"))
(unless (find-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDCOMMANDREQUEST")
  (make-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDCOMMANDREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDCOMMANDRESPONSE")
  (make-package "KORTEX_DRIVER::SENDSELECTEDJOINTSPEEDCOMMANDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendSelectedJointSpeedCommandRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SendSelectedJointSpeedCommandRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::JointSpeed :init))
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
    ;; kortex_driver/JointSpeed _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/JointSpeed _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/JointSpeed _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SendSelectedJointSpeedCommandResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SendSelectedJointSpeedCommandResponse
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

(defclass kortex_driver::SendSelectedJointSpeedCommand
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendSelectedJointSpeedCommand :md5sum-) "966eae21a761830257b0f20dfcd24c8e")
(setf (get kortex_driver::SendSelectedJointSpeedCommand :datatype-) "kortex_driver/SendSelectedJointSpeedCommand")
(setf (get kortex_driver::SendSelectedJointSpeedCommand :request) kortex_driver::SendSelectedJointSpeedCommandRequest)
(setf (get kortex_driver::SendSelectedJointSpeedCommand :response) kortex_driver::SendSelectedJointSpeedCommandResponse)

(defmethod kortex_driver::SendSelectedJointSpeedCommandRequest
  (:response () (instance kortex_driver::SendSelectedJointSpeedCommandResponse :init)))

(setf (get kortex_driver::SendSelectedJointSpeedCommandRequest :md5sum-) "966eae21a761830257b0f20dfcd24c8e")
(setf (get kortex_driver::SendSelectedJointSpeedCommandRequest :datatype-) "kortex_driver/SendSelectedJointSpeedCommandRequest")
(setf (get kortex_driver::SendSelectedJointSpeedCommandRequest :definition-)
      "JointSpeed input

================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SendSelectedJointSpeedCommandResponse :md5sum-) "966eae21a761830257b0f20dfcd24c8e")
(setf (get kortex_driver::SendSelectedJointSpeedCommandResponse :datatype-) "kortex_driver/SendSelectedJointSpeedCommandResponse")
(setf (get kortex_driver::SendSelectedJointSpeedCommandResponse :definition-)
      "JointSpeed input

================================================================================
MSG: kortex_driver/JointSpeed

uint32 joint_identifier
float32 value
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SendSelectedJointSpeedCommand "966eae21a761830257b0f20dfcd24c8e")


