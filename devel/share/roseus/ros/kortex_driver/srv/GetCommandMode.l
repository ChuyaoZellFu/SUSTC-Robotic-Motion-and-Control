;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetCommandMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetCommandMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCOMMANDMODE")
  (make-package "KORTEX_DRIVER::GETCOMMANDMODE"))
(unless (find-package "KORTEX_DRIVER::GETCOMMANDMODEREQUEST")
  (make-package "KORTEX_DRIVER::GETCOMMANDMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCOMMANDMODERESPONSE")
  (make-package "KORTEX_DRIVER::GETCOMMANDMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetCommandModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetCommandModeRequest
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

(defclass kortex_driver::GetCommandModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetCommandModeResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CommandModeInformation :init))
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
    ;; kortex_driver/CommandModeInformation _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CommandModeInformation _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CommandModeInformation _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetCommandMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetCommandMode :md5sum-) "6149f98eefd701a9f785c37c49326979")
(setf (get kortex_driver::GetCommandMode :datatype-) "kortex_driver/GetCommandMode")
(setf (get kortex_driver::GetCommandMode :request) kortex_driver::GetCommandModeRequest)
(setf (get kortex_driver::GetCommandMode :response) kortex_driver::GetCommandModeResponse)

(defmethod kortex_driver::GetCommandModeRequest
  (:response () (instance kortex_driver::GetCommandModeResponse :init)))

(setf (get kortex_driver::GetCommandModeRequest :md5sum-) "6149f98eefd701a9f785c37c49326979")
(setf (get kortex_driver::GetCommandModeRequest :datatype-) "kortex_driver/GetCommandModeRequest")
(setf (get kortex_driver::GetCommandModeRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CommandModeInformation output

================================================================================
MSG: kortex_driver/CommandModeInformation

")

(setf (get kortex_driver::GetCommandModeResponse :md5sum-) "6149f98eefd701a9f785c37c49326979")
(setf (get kortex_driver::GetCommandModeResponse :datatype-) "kortex_driver/GetCommandModeResponse")
(setf (get kortex_driver::GetCommandModeResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CommandModeInformation output

================================================================================
MSG: kortex_driver/CommandModeInformation

")



(provide :kortex_driver/GetCommandMode "6149f98eefd701a9f785c37c49326979")


