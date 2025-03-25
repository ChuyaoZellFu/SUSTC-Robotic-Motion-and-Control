;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DuplicateMapping)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DuplicateMapping (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DUPLICATEMAPPING")
  (make-package "KORTEX_DRIVER::DUPLICATEMAPPING"))
(unless (find-package "KORTEX_DRIVER::DUPLICATEMAPPINGREQUEST")
  (make-package "KORTEX_DRIVER::DUPLICATEMAPPINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::DUPLICATEMAPPINGRESPONSE")
  (make-package "KORTEX_DRIVER::DUPLICATEMAPPINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DuplicateMappingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DuplicateMappingRequest
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

(defclass kortex_driver::DuplicateMappingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DuplicateMappingResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::MappingHandle :init))
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
    ;; kortex_driver/MappingHandle _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MappingHandle _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MappingHandle _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DuplicateMapping
  :super ros::object
  :slots ())

(setf (get kortex_driver::DuplicateMapping :md5sum-) "bfcefbce8997010c78c9f62aadaabcab")
(setf (get kortex_driver::DuplicateMapping :datatype-) "kortex_driver/DuplicateMapping")
(setf (get kortex_driver::DuplicateMapping :request) kortex_driver::DuplicateMappingRequest)
(setf (get kortex_driver::DuplicateMapping :response) kortex_driver::DuplicateMappingResponse)

(defmethod kortex_driver::DuplicateMappingRequest
  (:response () (instance kortex_driver::DuplicateMappingResponse :init)))

(setf (get kortex_driver::DuplicateMappingRequest :md5sum-) "bfcefbce8997010c78c9f62aadaabcab")
(setf (get kortex_driver::DuplicateMappingRequest :datatype-) "kortex_driver/DuplicateMappingRequest")
(setf (get kortex_driver::DuplicateMappingRequest :definition-)
      "MappingHandle input

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
---
MappingHandle output

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
")

(setf (get kortex_driver::DuplicateMappingResponse :md5sum-) "bfcefbce8997010c78c9f62aadaabcab")
(setf (get kortex_driver::DuplicateMappingResponse :datatype-) "kortex_driver/DuplicateMappingResponse")
(setf (get kortex_driver::DuplicateMappingResponse :definition-)
      "MappingHandle input

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
---
MappingHandle output

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
")



(provide :kortex_driver/DuplicateMapping "bfcefbce8997010c78c9f62aadaabcab")


