;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetCommandMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetCommandMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETCOMMANDMODE")
  (make-package "KORTEX_DRIVER::SETCOMMANDMODE"))
(unless (find-package "KORTEX_DRIVER::SETCOMMANDMODEREQUEST")
  (make-package "KORTEX_DRIVER::SETCOMMANDMODEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETCOMMANDMODERESPONSE")
  (make-package "KORTEX_DRIVER::SETCOMMANDMODERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetCommandModeRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetCommandModeRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::CommandModeInformation :init))
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
    ;; kortex_driver/CommandModeInformation _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CommandModeInformation _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CommandModeInformation _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetCommandModeResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetCommandModeResponse
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

(defclass kortex_driver::SetCommandMode
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetCommandMode :md5sum-) "ca50afb242dba49984abe55c3e0b6692")
(setf (get kortex_driver::SetCommandMode :datatype-) "kortex_driver/SetCommandMode")
(setf (get kortex_driver::SetCommandMode :request) kortex_driver::SetCommandModeRequest)
(setf (get kortex_driver::SetCommandMode :response) kortex_driver::SetCommandModeResponse)

(defmethod kortex_driver::SetCommandModeRequest
  (:response () (instance kortex_driver::SetCommandModeResponse :init)))

(setf (get kortex_driver::SetCommandModeRequest :md5sum-) "ca50afb242dba49984abe55c3e0b6692")
(setf (get kortex_driver::SetCommandModeRequest :datatype-) "kortex_driver/SetCommandModeRequest")
(setf (get kortex_driver::SetCommandModeRequest :definition-)
      "CommandModeInformation input

================================================================================
MSG: kortex_driver/CommandModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetCommandModeResponse :md5sum-) "ca50afb242dba49984abe55c3e0b6692")
(setf (get kortex_driver::SetCommandModeResponse :datatype-) "kortex_driver/SetCommandModeResponse")
(setf (get kortex_driver::SetCommandModeResponse :definition-)
      "CommandModeInformation input

================================================================================
MSG: kortex_driver/CommandModeInformation

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetCommandMode "ca50afb242dba49984abe55c3e0b6692")


