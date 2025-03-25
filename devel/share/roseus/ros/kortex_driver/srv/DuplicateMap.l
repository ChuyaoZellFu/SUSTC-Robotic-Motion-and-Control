;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DuplicateMap)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DuplicateMap (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DUPLICATEMAP")
  (make-package "KORTEX_DRIVER::DUPLICATEMAP"))
(unless (find-package "KORTEX_DRIVER::DUPLICATEMAPREQUEST")
  (make-package "KORTEX_DRIVER::DUPLICATEMAPREQUEST"))
(unless (find-package "KORTEX_DRIVER::DUPLICATEMAPRESPONSE")
  (make-package "KORTEX_DRIVER::DUPLICATEMAPRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DuplicateMapRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DuplicateMapRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::MapHandle :init))
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
    ;; kortex_driver/MapHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MapHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MapHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DuplicateMapResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DuplicateMapResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::MapHandle :init))
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
    ;; kortex_driver/MapHandle _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MapHandle _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MapHandle _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DuplicateMap
  :super ros::object
  :slots ())

(setf (get kortex_driver::DuplicateMap :md5sum-) "bfcefbce8997010c78c9f62aadaabcab")
(setf (get kortex_driver::DuplicateMap :datatype-) "kortex_driver/DuplicateMap")
(setf (get kortex_driver::DuplicateMap :request) kortex_driver::DuplicateMapRequest)
(setf (get kortex_driver::DuplicateMap :response) kortex_driver::DuplicateMapResponse)

(defmethod kortex_driver::DuplicateMapRequest
  (:response () (instance kortex_driver::DuplicateMapResponse :init)))

(setf (get kortex_driver::DuplicateMapRequest :md5sum-) "bfcefbce8997010c78c9f62aadaabcab")
(setf (get kortex_driver::DuplicateMapRequest :datatype-) "kortex_driver/DuplicateMapRequest")
(setf (get kortex_driver::DuplicateMapRequest :definition-)
      "MapHandle input

================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
---
MapHandle output

================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
")

(setf (get kortex_driver::DuplicateMapResponse :md5sum-) "bfcefbce8997010c78c9f62aadaabcab")
(setf (get kortex_driver::DuplicateMapResponse :datatype-) "kortex_driver/DuplicateMapResponse")
(setf (get kortex_driver::DuplicateMapResponse :definition-)
      "MapHandle input

================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
---
MapHandle output

================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
")



(provide :kortex_driver/DuplicateMap "bfcefbce8997010c78c9f62aadaabcab")


