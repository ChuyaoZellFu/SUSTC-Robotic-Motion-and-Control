;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERHANDLE")
  (make-package "KORTEX_DRIVER::CONTROLLERHANDLE"))

(in-package "ROS")
;;//! \htmlinclude ControllerHandle.msg.html


(defclass kortex_driver::ControllerHandle
  :super ros::object
  :slots (_type _controller_identifier ))

(defmethod kortex_driver::ControllerHandle
  (:init
   (&key
    ((:type __type) 0)
    ((:controller_identifier __controller_identifier) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _controller_identifier (round __controller_identifier))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:controller_identifier
   (&optional __controller_identifier)
   (if __controller_identifier (setq _controller_identifier __controller_identifier)) _controller_identifier)
  (:serialization-length
   ()
   (+
    ;; uint32 _type
    4
    ;; uint32 _controller_identifier
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _type
       (write-long _type s)
     ;; uint32 _controller_identifier
       (write-long _controller_identifier s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _controller_identifier
     (setq _controller_identifier (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ControllerHandle :md5sum-) "e80ce64bff64a2583fb879d270960edd")
(setf (get kortex_driver::ControllerHandle :datatype-) "kortex_driver/ControllerHandle")
(setf (get kortex_driver::ControllerHandle :definition-)
      "
uint32 type
uint32 controller_identifier
")



(provide :kortex_driver/ControllerHandle "e80ce64bff64a2583fb879d270960edd")


