;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActionHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActionHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTIONHANDLE")
  (make-package "KORTEX_DRIVER::ACTIONHANDLE"))

(in-package "ROS")
;;//! \htmlinclude ActionHandle.msg.html


(defclass kortex_driver::ActionHandle
  :super ros::object
  :slots (_identifier _action_type _permission ))

(defmethod kortex_driver::ActionHandle
  (:init
   (&key
    ((:identifier __identifier) 0)
    ((:action_type __action_type) 0)
    ((:permission __permission) 0)
    )
   (send-super :init)
   (setq _identifier (round __identifier))
   (setq _action_type (round __action_type))
   (setq _permission (round __permission))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:action_type
   (&optional __action_type)
   (if __action_type (setq _action_type __action_type)) _action_type)
  (:permission
   (&optional __permission)
   (if __permission (setq _permission __permission)) _permission)
  (:serialization-length
   ()
   (+
    ;; uint32 _identifier
    4
    ;; uint32 _action_type
    4
    ;; uint32 _permission
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _identifier
       (write-long _identifier s)
     ;; uint32 _action_type
       (write-long _action_type s)
     ;; uint32 _permission
       (write-long _permission s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _identifier
     (setq _identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _action_type
     (setq _action_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _permission
     (setq _permission (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ActionHandle :md5sum-) "107d90c96b3183e90ce57fabb63c6f1f")
(setf (get kortex_driver::ActionHandle :datatype-) "kortex_driver/ActionHandle")
(setf (get kortex_driver::ActionHandle :definition-)
      "
uint32 identifier
uint32 action_type
uint32 permission
")



(provide :kortex_driver/ActionHandle "107d90c96b3183e90ce57fabb63c6f1f")


