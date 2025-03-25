;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MappingList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MappingList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPPINGLIST")
  (make-package "KORTEX_DRIVER::MAPPINGLIST"))

(in-package "ROS")
;;//! \htmlinclude MappingList.msg.html


(defclass kortex_driver::MappingList
  :super ros::object
  :slots (_mappings ))

(defmethod kortex_driver::MappingList
  (:init
   (&key
    ((:mappings __mappings) ())
    )
   (send-super :init)
   (setq _mappings __mappings)
   self)
  (:mappings
   (&rest __mappings)
   (if (keywordp (car __mappings))
       (send* _mappings __mappings)
     (progn
       (if __mappings (setq _mappings (car __mappings)))
       _mappings)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Mapping[] _mappings
    (apply #'+ (send-all _mappings :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Mapping[] _mappings
     (write-long (length _mappings) s)
     (dolist (elem _mappings)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Mapping[] _mappings
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mappings (let (r) (dotimes (i n) (push (instance kortex_driver::Mapping :init) r)) r))
     (dolist (elem- _mappings)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::MappingList :md5sum-) "bb960bc69e45bc26dd9e7ed2dbaeeaaf")
(setf (get kortex_driver::MappingList :datatype-) "kortex_driver/MappingList")
(setf (get kortex_driver::MappingList :definition-)
      "
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
uint32 permission
")



(provide :kortex_driver/MappingList "bb960bc69e45bc26dd9e7ed2dbaeeaaf")


