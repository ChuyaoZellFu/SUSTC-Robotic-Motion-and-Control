;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UpdateMapping)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UpdateMapping (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UPDATEMAPPING")
  (make-package "KORTEX_DRIVER::UPDATEMAPPING"))
(unless (find-package "KORTEX_DRIVER::UPDATEMAPPINGREQUEST")
  (make-package "KORTEX_DRIVER::UPDATEMAPPINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::UPDATEMAPPINGRESPONSE")
  (make-package "KORTEX_DRIVER::UPDATEMAPPINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::UpdateMappingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::UpdateMappingRequest
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

(defclass kortex_driver::UpdateMappingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::UpdateMappingResponse
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

(defclass kortex_driver::UpdateMapping
  :super ros::object
  :slots ())

(setf (get kortex_driver::UpdateMapping :md5sum-) "921c2b05776ccd75721af97241aa399d")
(setf (get kortex_driver::UpdateMapping :datatype-) "kortex_driver/UpdateMapping")
(setf (get kortex_driver::UpdateMapping :request) kortex_driver::UpdateMappingRequest)
(setf (get kortex_driver::UpdateMapping :response) kortex_driver::UpdateMappingResponse)

(defmethod kortex_driver::UpdateMappingRequest
  (:response () (instance kortex_driver::UpdateMappingResponse :init)))

(setf (get kortex_driver::UpdateMappingRequest :md5sum-) "921c2b05776ccd75721af97241aa399d")
(setf (get kortex_driver::UpdateMappingRequest :datatype-) "kortex_driver/UpdateMappingRequest")
(setf (get kortex_driver::UpdateMappingRequest :definition-)
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
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::UpdateMappingResponse :md5sum-) "921c2b05776ccd75721af97241aa399d")
(setf (get kortex_driver::UpdateMappingResponse :datatype-) "kortex_driver/UpdateMappingResponse")
(setf (get kortex_driver::UpdateMappingResponse :definition-)
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
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/UpdateMapping "921c2b05776ccd75721af97241aa399d")


