;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SwitchControlMapping)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SwitchControlMapping (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SWITCHCONTROLMAPPING")
  (make-package "KORTEX_DRIVER::SWITCHCONTROLMAPPING"))

(in-package "ROS")
;;//! \htmlinclude SwitchControlMapping.msg.html


(defclass kortex_driver::SwitchControlMapping
  :super ros::object
  :slots (_controller_identifier _map_group_handle _map_handle ))

(defmethod kortex_driver::SwitchControlMapping
  (:init
   (&key
    ((:controller_identifier __controller_identifier) 0)
    ((:map_group_handle __map_group_handle) (instance kortex_driver::MapGroupHandle :init))
    ((:map_handle __map_handle) (instance kortex_driver::MapHandle :init))
    )
   (send-super :init)
   (setq _controller_identifier (round __controller_identifier))
   (setq _map_group_handle __map_group_handle)
   (setq _map_handle __map_handle)
   self)
  (:controller_identifier
   (&optional __controller_identifier)
   (if __controller_identifier (setq _controller_identifier __controller_identifier)) _controller_identifier)
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
    ;; uint32 _controller_identifier
    4
    ;; kortex_driver/MapGroupHandle _map_group_handle
    (send _map_group_handle :serialization-length)
    ;; kortex_driver/MapHandle _map_handle
    (send _map_handle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _controller_identifier
       (write-long _controller_identifier s)
     ;; kortex_driver/MapGroupHandle _map_group_handle
       (send _map_group_handle :serialize s)
     ;; kortex_driver/MapHandle _map_handle
       (send _map_handle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _controller_identifier
     (setq _controller_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/MapGroupHandle _map_group_handle
     (send _map_group_handle :deserialize buf ptr-) (incf ptr- (send _map_group_handle :serialization-length))
   ;; kortex_driver/MapHandle _map_handle
     (send _map_handle :deserialize buf ptr-) (incf ptr- (send _map_handle :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::SwitchControlMapping :md5sum-) "4a7c904b5b1279f1ceda6c334529fa90")
(setf (get kortex_driver::SwitchControlMapping :datatype-) "kortex_driver/SwitchControlMapping")
(setf (get kortex_driver::SwitchControlMapping :definition-)
      "
uint32 controller_identifier
MapGroupHandle map_group_handle
MapHandle map_handle
================================================================================
MSG: kortex_driver/MapGroupHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/SwitchControlMapping "4a7c904b5b1279f1ceda6c334529fa90")


