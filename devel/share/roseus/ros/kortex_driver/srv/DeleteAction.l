;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteAction)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteAction (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETEACTION")
  (make-package "KORTEX_DRIVER::DELETEACTION"))
(unless (find-package "KORTEX_DRIVER::DELETEACTIONREQUEST")
  (make-package "KORTEX_DRIVER::DELETEACTIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETEACTIONRESPONSE")
  (make-package "KORTEX_DRIVER::DELETEACTIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteActionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteActionRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ActionHandle :init))
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
    ;; kortex_driver/ActionHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ActionHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ActionHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DeleteActionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteActionResponse
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

(defclass kortex_driver::DeleteAction
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteAction :md5sum-) "39696246fa7132aebfa0097dedbf54c1")
(setf (get kortex_driver::DeleteAction :datatype-) "kortex_driver/DeleteAction")
(setf (get kortex_driver::DeleteAction :request) kortex_driver::DeleteActionRequest)
(setf (get kortex_driver::DeleteAction :response) kortex_driver::DeleteActionResponse)

(defmethod kortex_driver::DeleteActionRequest
  (:response () (instance kortex_driver::DeleteActionResponse :init)))

(setf (get kortex_driver::DeleteActionRequest :md5sum-) "39696246fa7132aebfa0097dedbf54c1")
(setf (get kortex_driver::DeleteActionRequest :datatype-) "kortex_driver/DeleteActionRequest")
(setf (get kortex_driver::DeleteActionRequest :definition-)
      "ActionHandle input

================================================================================
MSG: kortex_driver/ActionHandle

uint32 identifier
uint32 action_type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteActionResponse :md5sum-) "39696246fa7132aebfa0097dedbf54c1")
(setf (get kortex_driver::DeleteActionResponse :datatype-) "kortex_driver/DeleteActionResponse")
(setf (get kortex_driver::DeleteActionResponse :definition-)
      "ActionHandle input

================================================================================
MSG: kortex_driver/ActionHandle

uint32 identifier
uint32 action_type
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteAction "39696246fa7132aebfa0097dedbf54c1")


