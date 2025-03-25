;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteMapping)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteMapping (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETEMAPPING")
  (make-package "KORTEX_DRIVER::DELETEMAPPING"))
(unless (find-package "KORTEX_DRIVER::DELETEMAPPINGREQUEST")
  (make-package "KORTEX_DRIVER::DELETEMAPPINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETEMAPPINGRESPONSE")
  (make-package "KORTEX_DRIVER::DELETEMAPPINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteMappingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteMappingRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::MappingHandle :init))
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
    ;; kortex_driver/MappingHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MappingHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MappingHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DeleteMappingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteMappingResponse
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

(defclass kortex_driver::DeleteMapping
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteMapping :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteMapping :datatype-) "kortex_driver/DeleteMapping")
(setf (get kortex_driver::DeleteMapping :request) kortex_driver::DeleteMappingRequest)
(setf (get kortex_driver::DeleteMapping :response) kortex_driver::DeleteMappingResponse)

(defmethod kortex_driver::DeleteMappingRequest
  (:response () (instance kortex_driver::DeleteMappingResponse :init)))

(setf (get kortex_driver::DeleteMappingRequest :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteMappingRequest :datatype-) "kortex_driver/DeleteMappingRequest")
(setf (get kortex_driver::DeleteMappingRequest :definition-)
      "MappingHandle input

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteMappingResponse :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteMappingResponse :datatype-) "kortex_driver/DeleteMappingResponse")
(setf (get kortex_driver::DeleteMappingResponse :definition-)
      "MappingHandle input

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteMapping "04dfaeca45772f660e0913aa84774e13")


