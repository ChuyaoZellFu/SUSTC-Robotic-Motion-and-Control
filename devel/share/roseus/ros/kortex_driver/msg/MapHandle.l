;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MapHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MapHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPHANDLE")
  (make-package "KORTEX_DRIVER::MAPHANDLE"))

(in-package "ROS")
;;//! \htmlinclude MapHandle.msg.html


(defclass kortex_driver::MapHandle
  :super ros::object
  :slots (_identifier _permission ))

(defmethod kortex_driver::MapHandle
  (:init
   (&key
    ((:identifier __identifier) 0)
    ((:permission __permission) 0)
    )
   (send-super :init)
   (setq _identifier (round __identifier))
   (setq _permission (round __permission))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:permission
   (&optional __permission)
   (if __permission (setq _permission __permission)) _permission)
  (:serialization-length
   ()
   (+
    ;; uint32 _identifier
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
     ;; uint32 _permission
       (write-long _permission s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _identifier
     (setq _identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _permission
     (setq _permission (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::MapHandle :md5sum-) "10d841c6abc3fd3596d9b10510d50074")
(setf (get kortex_driver::MapHandle :datatype-) "kortex_driver/MapHandle")
(setf (get kortex_driver::MapHandle :definition-)
      "
uint32 identifier
uint32 permission
")



(provide :kortex_driver/MapHandle "10d841c6abc3fd3596d9b10510d50074")


