;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MapGroup)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MapGroup (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPGROUP")
  (make-package "KORTEX_DRIVER::MAPGROUP"))

(in-package "ROS")
;;//! \htmlinclude MapGroup.msg.html


(defclass kortex_driver::MapGroup
  :super ros::object
  :slots (_group_handle _name _related_mapping_handle _parent_group_handle _children_map_group_handles _map_handles _application_data ))

(defmethod kortex_driver::MapGroup
  (:init
   (&key
    ((:group_handle __group_handle) (instance kortex_driver::MapGroupHandle :init))
    ((:name __name) "")
    ((:related_mapping_handle __related_mapping_handle) (instance kortex_driver::MappingHandle :init))
    ((:parent_group_handle __parent_group_handle) (instance kortex_driver::MapGroupHandle :init))
    ((:children_map_group_handles __children_map_group_handles) ())
    ((:map_handles __map_handles) ())
    ((:application_data __application_data) "")
    )
   (send-super :init)
   (setq _group_handle __group_handle)
   (setq _name (string __name))
   (setq _related_mapping_handle __related_mapping_handle)
   (setq _parent_group_handle __parent_group_handle)
   (setq _children_map_group_handles __children_map_group_handles)
   (setq _map_handles __map_handles)
   (setq _application_data (string __application_data))
   self)
  (:group_handle
   (&rest __group_handle)
   (if (keywordp (car __group_handle))
       (send* _group_handle __group_handle)
     (progn
       (if __group_handle (setq _group_handle (car __group_handle)))
       _group_handle)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:related_mapping_handle
   (&rest __related_mapping_handle)
   (if (keywordp (car __related_mapping_handle))
       (send* _related_mapping_handle __related_mapping_handle)
     (progn
       (if __related_mapping_handle (setq _related_mapping_handle (car __related_mapping_handle)))
       _related_mapping_handle)))
  (:parent_group_handle
   (&rest __parent_group_handle)
   (if (keywordp (car __parent_group_handle))
       (send* _parent_group_handle __parent_group_handle)
     (progn
       (if __parent_group_handle (setq _parent_group_handle (car __parent_group_handle)))
       _parent_group_handle)))
  (:children_map_group_handles
   (&rest __children_map_group_handles)
   (if (keywordp (car __children_map_group_handles))
       (send* _children_map_group_handles __children_map_group_handles)
     (progn
       (if __children_map_group_handles (setq _children_map_group_handles (car __children_map_group_handles)))
       _children_map_group_handles)))
  (:map_handles
   (&rest __map_handles)
   (if (keywordp (car __map_handles))
       (send* _map_handles __map_handles)
     (progn
       (if __map_handles (setq _map_handles (car __map_handles)))
       _map_handles)))
  (:application_data
   (&optional __application_data)
   (if __application_data (setq _application_data __application_data)) _application_data)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/MapGroupHandle _group_handle
    (send _group_handle :serialization-length)
    ;; string _name
    4 (length _name)
    ;; kortex_driver/MappingHandle _related_mapping_handle
    (send _related_mapping_handle :serialization-length)
    ;; kortex_driver/MapGroupHandle _parent_group_handle
    (send _parent_group_handle :serialization-length)
    ;; kortex_driver/MapGroupHandle[] _children_map_group_handles
    (apply #'+ (send-all _children_map_group_handles :serialization-length)) 4
    ;; kortex_driver/MapHandle[] _map_handles
    (apply #'+ (send-all _map_handles :serialization-length)) 4
    ;; string _application_data
    4 (length _application_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MapGroupHandle _group_handle
       (send _group_handle :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; kortex_driver/MappingHandle _related_mapping_handle
       (send _related_mapping_handle :serialize s)
     ;; kortex_driver/MapGroupHandle _parent_group_handle
       (send _parent_group_handle :serialize s)
     ;; kortex_driver/MapGroupHandle[] _children_map_group_handles
     (write-long (length _children_map_group_handles) s)
     (dolist (elem _children_map_group_handles)
       (send elem :serialize s)
       )
     ;; kortex_driver/MapHandle[] _map_handles
     (write-long (length _map_handles) s)
     (dolist (elem _map_handles)
       (send elem :serialize s)
       )
     ;; string _application_data
       (write-long (length _application_data) s) (princ _application_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/MapGroupHandle _group_handle
     (send _group_handle :deserialize buf ptr-) (incf ptr- (send _group_handle :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kortex_driver/MappingHandle _related_mapping_handle
     (send _related_mapping_handle :deserialize buf ptr-) (incf ptr- (send _related_mapping_handle :serialization-length))
   ;; kortex_driver/MapGroupHandle _parent_group_handle
     (send _parent_group_handle :deserialize buf ptr-) (incf ptr- (send _parent_group_handle :serialization-length))
   ;; kortex_driver/MapGroupHandle[] _children_map_group_handles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _children_map_group_handles (let (r) (dotimes (i n) (push (instance kortex_driver::MapGroupHandle :init) r)) r))
     (dolist (elem- _children_map_group_handles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/MapHandle[] _map_handles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_handles (let (r) (dotimes (i n) (push (instance kortex_driver::MapHandle :init) r)) r))
     (dolist (elem- _map_handles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string _application_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _application_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::MapGroup :md5sum-) "f19116dedc7aefc01a2fc92e64e79046")
(setf (get kortex_driver::MapGroup :datatype-) "kortex_driver/MapGroup")
(setf (get kortex_driver::MapGroup :definition-)
      "
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



(provide :kortex_driver/MapGroup "f19116dedc7aefc01a2fc92e64e79046")


