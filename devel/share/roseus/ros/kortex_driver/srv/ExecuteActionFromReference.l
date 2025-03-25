;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ExecuteActionFromReference)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ExecuteActionFromReference (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::EXECUTEACTIONFROMREFERENCE")
  (make-package "KORTEX_DRIVER::EXECUTEACTIONFROMREFERENCE"))
(unless (find-package "KORTEX_DRIVER::EXECUTEACTIONFROMREFERENCEREQUEST")
  (make-package "KORTEX_DRIVER::EXECUTEACTIONFROMREFERENCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::EXECUTEACTIONFROMREFERENCERESPONSE")
  (make-package "KORTEX_DRIVER::EXECUTEACTIONFROMREFERENCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ExecuteActionFromReferenceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ExecuteActionFromReferenceRequest
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

(defclass kortex_driver::ExecuteActionFromReferenceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ExecuteActionFromReferenceResponse
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

(defclass kortex_driver::ExecuteActionFromReference
  :super ros::object
  :slots ())

(setf (get kortex_driver::ExecuteActionFromReference :md5sum-) "39696246fa7132aebfa0097dedbf54c1")
(setf (get kortex_driver::ExecuteActionFromReference :datatype-) "kortex_driver/ExecuteActionFromReference")
(setf (get kortex_driver::ExecuteActionFromReference :request) kortex_driver::ExecuteActionFromReferenceRequest)
(setf (get kortex_driver::ExecuteActionFromReference :response) kortex_driver::ExecuteActionFromReferenceResponse)

(defmethod kortex_driver::ExecuteActionFromReferenceRequest
  (:response () (instance kortex_driver::ExecuteActionFromReferenceResponse :init)))

(setf (get kortex_driver::ExecuteActionFromReferenceRequest :md5sum-) "39696246fa7132aebfa0097dedbf54c1")
(setf (get kortex_driver::ExecuteActionFromReferenceRequest :datatype-) "kortex_driver/ExecuteActionFromReferenceRequest")
(setf (get kortex_driver::ExecuteActionFromReferenceRequest :definition-)
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

(setf (get kortex_driver::ExecuteActionFromReferenceResponse :md5sum-) "39696246fa7132aebfa0097dedbf54c1")
(setf (get kortex_driver::ExecuteActionFromReferenceResponse :datatype-) "kortex_driver/ExecuteActionFromReferenceResponse")
(setf (get kortex_driver::ExecuteActionFromReferenceResponse :definition-)
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



(provide :kortex_driver/ExecuteActionFromReference "39696246fa7132aebfa0097dedbf54c1")


