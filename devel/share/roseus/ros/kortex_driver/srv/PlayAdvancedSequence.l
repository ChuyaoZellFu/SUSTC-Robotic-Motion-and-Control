;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PlayAdvancedSequence)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PlayAdvancedSequence (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PLAYADVANCEDSEQUENCE")
  (make-package "KORTEX_DRIVER::PLAYADVANCEDSEQUENCE"))
(unless (find-package "KORTEX_DRIVER::PLAYADVANCEDSEQUENCEREQUEST")
  (make-package "KORTEX_DRIVER::PLAYADVANCEDSEQUENCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::PLAYADVANCEDSEQUENCERESPONSE")
  (make-package "KORTEX_DRIVER::PLAYADVANCEDSEQUENCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::PlayAdvancedSequenceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::PlayAdvancedSequenceRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::AdvancedSequenceHandle :init))
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
    ;; kortex_driver/AdvancedSequenceHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/AdvancedSequenceHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/AdvancedSequenceHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::PlayAdvancedSequenceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::PlayAdvancedSequenceResponse
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

(defclass kortex_driver::PlayAdvancedSequence
  :super ros::object
  :slots ())

(setf (get kortex_driver::PlayAdvancedSequence :md5sum-) "ff3d79c2f78fa637d54a7284e0052ea0")
(setf (get kortex_driver::PlayAdvancedSequence :datatype-) "kortex_driver/PlayAdvancedSequence")
(setf (get kortex_driver::PlayAdvancedSequence :request) kortex_driver::PlayAdvancedSequenceRequest)
(setf (get kortex_driver::PlayAdvancedSequence :response) kortex_driver::PlayAdvancedSequenceResponse)

(defmethod kortex_driver::PlayAdvancedSequenceRequest
  (:response () (instance kortex_driver::PlayAdvancedSequenceResponse :init)))

(setf (get kortex_driver::PlayAdvancedSequenceRequest :md5sum-) "ff3d79c2f78fa637d54a7284e0052ea0")
(setf (get kortex_driver::PlayAdvancedSequenceRequest :datatype-) "kortex_driver/PlayAdvancedSequenceRequest")
(setf (get kortex_driver::PlayAdvancedSequenceRequest :definition-)
      "AdvancedSequenceHandle input

================================================================================
MSG: kortex_driver/AdvancedSequenceHandle

SequenceHandle handle
bool in_loop
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::PlayAdvancedSequenceResponse :md5sum-) "ff3d79c2f78fa637d54a7284e0052ea0")
(setf (get kortex_driver::PlayAdvancedSequenceResponse :datatype-) "kortex_driver/PlayAdvancedSequenceResponse")
(setf (get kortex_driver::PlayAdvancedSequenceResponse :definition-)
      "AdvancedSequenceHandle input

================================================================================
MSG: kortex_driver/AdvancedSequenceHandle

SequenceHandle handle
bool in_loop
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/PlayAdvancedSequence "ff3d79c2f78fa637d54a7284e0052ea0")


