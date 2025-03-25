;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MapGroupList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MapGroupList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPGROUPLIST")
  (make-package "KORTEX_DRIVER::MAPGROUPLIST"))

(in-package "ROS")
;;//! \htmlinclude MapGroupList.msg.html


(defclass kortex_driver::MapGroupList
  :super ros::object
  :slots (_map_groups ))

(defmethod kortex_driver::MapGroupList
  (:init
   (&key
    ((:map_groups __map_groups) ())
    )
   (send-super :init)
   (setq _map_groups __map_groups)
   self)
  (:map_groups
   (&rest __map_groups)
   (if (keywordp (car __map_groups))
       (send* _map_groups __map_groups)
     (progn
       (if __map_groups (setq _map_groups (car __map_groups)))
       _map_groups)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/MapGroup[] _map_groups
    (apply #'+ (send-all _map_groups :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MapGroup[] _map_groups
     (write-long (length _map_groups) s)
     (dolist (elem _map_groups)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MapGroup[] _map_groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_groups (let (r) (dotimes (i n) (push (instance kortex_driver::MapGroup :init) r)) r))
     (dolist (elem- _map_groups)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::MapGroupList :md5sum-) "60049e6988ccab2e80e3d3cdf02a81da")
(setf (get kortex_driver::MapGroupList :datatype-) "kortex_driver/MapGroupList")
(setf (get kortex_driver::MapGroupList :definition-)
      "
MapGroup[] map_groups
================================================================================
MSG: kortex_driver/MapGroup

MapGroupHandle group_handle
string name
MappingHandle related_mapping_handle
MapGroupHandle parent_group_handle
MapGroupHandle[] children_map_group_handles
MapHandle[] map_handles
string application_data
================================================================================
MSG: kortex_driver/MapGroupHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/MapGroupList "60049e6988ccab2e80e3d3cdf02a81da")


