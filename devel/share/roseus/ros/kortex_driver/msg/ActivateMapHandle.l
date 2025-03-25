;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActivateMapHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActivateMapHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTIVATEMAPHANDLE")
  (make-package "KORTEX_DRIVER::ACTIVATEMAPHANDLE"))

(in-package "ROS")
;;//! \htmlinclude ActivateMapHandle.msg.html


(defclass kortex_driver::ActivateMapHandle
  :super ros::object
  :slots (_mapping_handle _map_group_handle _map_handle ))

(defmethod kortex_driver::ActivateMapHandle
  (:init
   (&key
    ((:mapping_handle __mapping_handle) (instance kortex_driver::MappingHandle :init))
    ((:map_group_handle __map_group_handle) (instance kortex_driver::MapGroupHandle :init))
    ((:map_handle __map_handle) (instance kortex_driver::MapHandle :init))
    )
   (send-super :init)
   (setq _mapping_handle __mapping_handle)
   (setq _map_group_handle __map_group_handle)
   (setq _map_handle __map_handle)
   self)
  (:mapping_handle
   (&rest __mapping_handle)
   (if (keywordp (car __mapping_handle))
       (send* _mapping_handle __mapping_handle)
     (progn
       (if __mapping_handle (setq _mapping_handle (car __mapping_handle)))
       _mapping_handle)))
  (:map_group_handle
   (&rest __map_group_handle)
   (if (keywordp (car __map_group_handle))
       (send* _map_group_handle __map_group_handle)
     (progn
       (if __map_group_handle (setq _map_group_handle (car __map_group_handle)))
       _map_group_handle)))
  (:map_handle
   (&rest __map_handle)
   (if (keywordp (car __map_handle))
       (send* _map_handle __map_handle)
     (progn
       (if __map_handle (setq _map_handle (car __map_handle)))
       _map_handle)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/MappingHandle _mapping_handle
    (send _mapping_handle :serialization-length)
    ;; kortex_driver/MapGroupHandle _map_group_handle
    (send _map_group_handle :serialization-length)
    ;; kortex_driver/MapHandle _map_handle
    (send _map_handle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MappingHandle _mapping_handle
       (send _mapping_handle :serialize s)
     ;; kortex_driver/MapGroupHandle _map_group_handle
       (send _map_group_handle :serialize s)
     ;; kortex_driver/MapHandle _map_handle
       (send _map_handle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MappingHandle _mapping_handle
     (send _mapping_handle :deserialize buf ptr-) (incf ptr- (send _mapping_handle :serialization-length))
   ;; kortex_driver/MapGroupHandle _map_group_handle
     (send _map_group_handle :deserialize buf ptr-) (incf ptr- (send _map_group_handle :serialization-length))
   ;; kortex_driver/MapHandle _map_handle
     (send _map_handle :deserialize buf ptr-) (incf ptr- (send _map_handle :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ActivateMapHandle :md5sum-) "3f76b24edc6fa7ed6cb77b6c5ee77745")
(setf (get kortex_driver::ActivateMapHandle :datatype-) "kortex_driver/ActivateMapHandle")
(setf (get kortex_driver::ActivateMapHandle :definition-)
      "
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
uint32 permission
")



(provide :kortex_driver/ActivateMapHandle "3f76b24edc6fa7ed6cb77b6c5ee77745")


