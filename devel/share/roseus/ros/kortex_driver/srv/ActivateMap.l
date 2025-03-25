;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActivateMap)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActivateMap (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTIVATEMAP")
  (make-package "KORTEX_DRIVER::ACTIVATEMAP"))
(unless (find-package "KORTEX_DRIVER::ACTIVATEMAPREQUEST")
  (make-package "KORTEX_DRIVER::ACTIVATEMAPREQUEST"))
(unless (find-package "KORTEX_DRIVER::ACTIVATEMAPRESPONSE")
  (make-package "KORTEX_DRIVER::ACTIVATEMAPRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ActivateMapRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ActivateMapRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ActivateMapHandle :init))
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
    ;; kortex_driver/ActivateMapHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ActivateMapHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ActivateMapHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ActivateMapResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ActivateMapResponse
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

(defclass kortex_driver::ActivateMap
  :super ros::object
  :slots ())

(setf (get kortex_driver::ActivateMap :md5sum-) "48d813b9a4f09a106ff67b248afca7e9")
(setf (get kortex_driver::ActivateMap :datatype-) "kortex_driver/ActivateMap")
(setf (get kortex_driver::ActivateMap :request) kortex_driver::ActivateMapRequest)
(setf (get kortex_driver::ActivateMap :response) kortex_driver::ActivateMapResponse)

(defmethod kortex_driver::ActivateMapRequest
  (:response () (instance kortex_driver::ActivateMapResponse :init)))

(setf (get kortex_driver::ActivateMapRequest :md5sum-) "48d813b9a4f09a106ff67b248afca7e9")
(setf (get kortex_driver::ActivateMapRequest :datatype-) "kortex_driver/ActivateMapRequest")
(setf (get kortex_driver::ActivateMapRequest :definition-)
      "ActivateMapHandle input

================================================================================
MSG: kortex_driver/ActivateMapHandle

MappingHandle mapping_handle
MapGroupHandle map_group_handle
MapHandle map_handle
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

(setf (get kortex_driver::ActivateMapResponse :md5sum-) "48d813b9a4f09a106ff67b248afca7e9")
(setf (get kortex_driver::ActivateMapResponse :datatype-) "kortex_driver/ActivateMapResponse")
(setf (get kortex_driver::ActivateMapResponse :definition-)
      "ActivateMapHandle input

================================================================================
MSG: kortex_driver/ActivateMapHandle

MappingHandle mapping_handle
MapGroupHandle map_group_handle
MapHandle map_handle
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



(provide :kortex_driver/ActivateMap "48d813b9a4f09a106ff67b248afca7e9")


