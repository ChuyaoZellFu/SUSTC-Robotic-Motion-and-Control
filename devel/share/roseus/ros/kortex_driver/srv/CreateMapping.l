;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CreateMapping)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CreateMapping (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CREATEMAPPING")
  (make-package "KORTEX_DRIVER::CREATEMAPPING"))
(unless (find-package "KORTEX_DRIVER::CREATEMAPPINGREQUEST")
  (make-package "KORTEX_DRIVER::CREATEMAPPINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::CREATEMAPPINGRESPONSE")
  (make-package "KORTEX_DRIVER::CREATEMAPPINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::CreateMappingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::CreateMappingRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Mapping :init))
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
    ;; kortex_driver/Mapping _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Mapping _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Mapping _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::CreateMappingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::CreateMappingResponse
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

(defclass kortex_driver::CreateMapping
  :super ros::object
  :slots ())

(setf (get kortex_driver::CreateMapping :md5sum-) "1672eeb2532b125a8c2350096e2628b8")
(setf (get kortex_driver::CreateMapping :datatype-) "kortex_driver/CreateMapping")
(setf (get kortex_driver::CreateMapping :request) kortex_driver::CreateMappingRequest)
(setf (get kortex_driver::CreateMapping :response) kortex_driver::CreateMappingResponse)

(defmethod kortex_driver::CreateMappingRequest
  (:response () (instance kortex_driver::CreateMappingResponse :init)))

(setf (get kortex_driver::CreateMappingRequest :md5sum-) "1672eeb2532b125a8c2350096e2628b8")
(setf (get kortex_driver::CreateMappingRequest :datatype-) "kortex_driver/CreateMappingRequest")
(setf (get kortex_driver::CreateMappingRequest :definition-)
      "Mapping input

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
---
MappingHandle output

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
")

(setf (get kortex_driver::CreateMappingResponse :md5sum-) "1672eeb2532b125a8c2350096e2628b8")
(setf (get kortex_driver::CreateMappingResponse :datatype-) "kortex_driver/CreateMappingResponse")
(setf (get kortex_driver::CreateMappingResponse :definition-)
      "Mapping input

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
---
MappingHandle output

================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
")



(provide :kortex_driver/CreateMapping "1672eeb2532b125a8c2350096e2628b8")


