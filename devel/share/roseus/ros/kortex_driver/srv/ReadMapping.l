;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadMapping)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadMapping (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READMAPPING")
  (make-package "KORTEX_DRIVER::READMAPPING"))
(unless (find-package "KORTEX_DRIVER::READMAPPINGREQUEST")
  (make-package "KORTEX_DRIVER::READMAPPINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::READMAPPINGRESPONSE")
  (make-package "KORTEX_DRIVER::READMAPPINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadMappingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadMappingRequest
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

(defclass kortex_driver::ReadMappingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadMappingResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Mapping :init))
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
    ;; kortex_driver/Mapping _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Mapping _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Mapping _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadMapping
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadMapping :md5sum-) "a098ab486d258af0a3206d3f4ae9d15c")
(setf (get kortex_driver::ReadMapping :datatype-) "kortex_driver/ReadMapping")
(setf (get kortex_driver::ReadMapping :request) kortex_driver::ReadMappingRequest)
(setf (get kortex_driver::ReadMapping :response) kortex_driver::ReadMappingResponse)

(defmethod kortex_driver::ReadMappingRequest
  (:response () (instance kortex_driver::ReadMappingResponse :init)))

(setf (get kortex_driver::ReadMappingRequest :md5sum-) "a098ab486d258af0a3206d3f4ae9d15c")
(setf (get kortex_driver::ReadMappingRequest :datatype-) "kortex_driver/ReadMappingRequest")
(setf (get kortex_driver::ReadMappingRequest :definition-)
      "MappingHandle input

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
---
Mapping output

================================================================================
MSG: kortex_driver/Mapping

MappingHandle handle
string name
uint32 controller_identifier
MapGroupHandle active_map_group_handle
MapGroupHandle[] map_group_handles
MapHandle active_map_handle
MapHandle[] map_handles
string application_data
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapGroupHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
")

(setf (get kortex_driver::ReadMappingResponse :md5sum-) "a098ab486d258af0a3206d3f4ae9d15c")
(setf (get kortex_driver::ReadMappingResponse :datatype-) "kortex_driver/ReadMappingResponse")
(setf (get kortex_driver::ReadMappingResponse :definition-)
      "MappingHandle input

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
---
Mapping output

================================================================================
MSG: kortex_driver/Mapping

MappingHandle handle
string name
uint32 controller_identifier
MapGroupHandle active_map_group_handle
MapGroupHandle[] map_group_handles
MapHandle active_map_handle
MapHandle[] map_handles
string application_data
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapGroupHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
")



(provide :kortex_driver/ReadMapping "a098ab486d258af0a3206d3f4ae9d15c")


