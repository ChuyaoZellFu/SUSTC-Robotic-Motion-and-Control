;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ReadAllMappings)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ReadAllMappings (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::READALLMAPPINGS")
  (make-package "KORTEX_DRIVER::READALLMAPPINGS"))
(unless (find-package "KORTEX_DRIVER::READALLMAPPINGSREQUEST")
  (make-package "KORTEX_DRIVER::READALLMAPPINGSREQUEST"))
(unless (find-package "KORTEX_DRIVER::READALLMAPPINGSRESPONSE")
  (make-package "KORTEX_DRIVER::READALLMAPPINGSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ReadAllMappingsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ReadAllMappingsRequest
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

(defclass kortex_driver::ReadAllMappingsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ReadAllMappingsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::MappingList :init))
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
    ;; kortex_driver/MappingList _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MappingList _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MappingList _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ReadAllMappings
  :super ros::object
  :slots ())

(setf (get kortex_driver::ReadAllMappings :md5sum-) "b5feb1f35cacb5c11e26808022e25f5c")
(setf (get kortex_driver::ReadAllMappings :datatype-) "kortex_driver/ReadAllMappings")
(setf (get kortex_driver::ReadAllMappings :request) kortex_driver::ReadAllMappingsRequest)
(setf (get kortex_driver::ReadAllMappings :response) kortex_driver::ReadAllMappingsResponse)

(defmethod kortex_driver::ReadAllMappingsRequest
  (:response () (instance kortex_driver::ReadAllMappingsResponse :init)))

(setf (get kortex_driver::ReadAllMappingsRequest :md5sum-) "b5feb1f35cacb5c11e26808022e25f5c")
(setf (get kortex_driver::ReadAllMappingsRequest :datatype-) "kortex_driver/ReadAllMappingsRequest")
(setf (get kortex_driver::ReadAllMappingsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
MappingList output

================================================================================
MSG: kortex_driver/MappingList

Mapping[] mappings
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

(setf (get kortex_driver::ReadAllMappingsResponse :md5sum-) "b5feb1f35cacb5c11e26808022e25f5c")
(setf (get kortex_driver::ReadAllMappingsResponse :datatype-) "kortex_driver/ReadAllMappingsResponse")
(setf (get kortex_driver::ReadAllMappingsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
MappingList output

================================================================================
MSG: kortex_driver/MappingList

Mapping[] mappings
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



(provide :kortex_driver/ReadAllMappings "b5feb1f35cacb5c11e26808022e25f5c")


