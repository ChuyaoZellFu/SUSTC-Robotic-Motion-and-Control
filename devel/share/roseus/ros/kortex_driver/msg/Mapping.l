;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Mapping)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Mapping (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPPING")
  (make-package "KORTEX_DRIVER::MAPPING"))

(in-package "ROS")
;;//! \htmlinclude Mapping.msg.html


(defclass kortex_driver::Mapping
  :super ros::object
  :slots (_handle _name _controller_identifier _active_map_group_handle _map_group_handles _active_map_handle _map_handles _application_data ))

(defmethod kortex_driver::Mapping
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::MappingHandle :init))
    ((:name __name) "")
    ((:controller_identifier __controller_identifier) 0)
    ((:active_map_group_handle __active_map_group_handle) (instance kortex_driver::MapGroupHandle :init))
    ((:map_group_handles __map_group_handles) ())
    ((:active_map_handle __active_map_handle) (instance kortex_driver::MapHandle :init))
    ((:map_handles __map_handles) ())
    ((:application_data __application_data) "")
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _name (string __name))
   (setq _controller_identifier (round __controller_identifier))
   (setq _active_map_group_handle __active_map_group_handle)
   (setq _map_group_handles __map_group_handles)
   (setq _active_map_handle __active_map_handle)
   (setq _map_handles __map_handles)
   (setq _application_data (string __application_data))
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:controller_identifier
   (&optional __controller_identifier)
   (if __controller_identifier (setq _controller_identifier __controller_identifier)) _controller_identifier)
  (:active_map_group_handle
   (&rest __active_map_group_handle)
   (if (keywordp (car __active_map_group_handle))
       (send* _active_map_group_handle __active_map_group_handle)
     (progn
       (if __active_map_group_handle (setq _active_map_group_handle (car __active_map_group_handle)))
       _active_map_group_handle)))
  (:map_group_handles
   (&rest __map_group_handles)
   (if (keywordp (car __map_group_handles))
       (send* _map_group_handles __map_group_handles)
     (progn
       (if __map_group_handles (setq _map_group_handles (car __map_group_handles)))
       _map_group_handles)))
  (:active_map_handle
   (&rest __active_map_handle)
   (if (keywordp (car __active_map_handle))
       (send* _active_map_handle __active_map_handle)
     (progn
       (if __active_map_handle (setq _active_map_handle (car __active_map_handle)))
       _active_map_handle)))
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
    ;; kortex_driver/MappingHandle _handle
    (send _handle :serialization-length)
    ;; string _name
    4 (length _name)
    ;; uint32 _controller_identifier
    4
    ;; kortex_driver/MapGroupHandle _active_map_group_handle
    (send _active_map_group_handle :serialization-length)
    ;; kortex_driver/MapGroupHandle[] _map_group_handles
    (apply #'+ (send-all _map_group_handles :serialization-length)) 4
    ;; kortex_driver/MapHandle _active_map_handle
    (send _active_map_handle :serialization-length)
    ;; kortex_driver/MapHandle[] _map_handles
    (apply #'+ (send-all _map_handles :serialization-length)) 4
    ;; string _application_data
    4 (length _application_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/MappingHandle _handle
       (send _handle :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint32 _controller_identifier
       (write-long _controller_identifier s)
     ;; kortex_driver/MapGroupHandle _active_map_group_handle
       (send _active_map_group_handle :serialize s)
     ;; kortex_driver/MapGroupHandle[] _map_group_handles
     (write-long (length _map_group_handles) s)
     (dolist (elem _map_group_handles)
       (send elem :serialize s)
       )
     ;; kortex_driver/MapHandle _active_map_handle
       (send _active_map_handle :serialize s)
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
   ;; kortex_driver/MappingHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _controller_identifier
     (setq _controller_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/MapGroupHandle _active_map_group_handle
     (send _active_map_group_handle :deserialize buf ptr-) (incf ptr- (send _active_map_group_handle :serialization-length))
   ;; kortex_driver/MapGroupHandle[] _map_group_handles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map_group_handles (let (r) (dotimes (i n) (push (instance kortex_driver::MapGroupHandle :init) r)) r))
     (dolist (elem- _map_group_handles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/MapHandle _active_map_handle
     (send _active_map_handle :deserialize buf ptr-) (incf ptr- (send _active_map_handle :serialization-length))
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

(setf (get kortex_driver::Mapping :md5sum-) "fda005d3531fdfb82d459bb0078389d2")
(setf (get kortex_driver::Mapping :datatype-) "kortex_driver/Mapping")
(setf (get kortex_driver::Mapping :definition-)
      "
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



(provide :kortex_driver/Mapping "fda005d3531fdfb82d459bb0078389d2")


