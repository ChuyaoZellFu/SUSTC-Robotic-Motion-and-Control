;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::NetworkHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'NetworkHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::NETWORKHANDLE")
  (make-package "KORTEX_DRIVER::NETWORKHANDLE"))

(in-package "ROS")
;;//! \htmlinclude NetworkHandle.msg.html


(defclass kortex_driver::NetworkHandle
  :super ros::object
  :slots (_type ))

(defmethod kortex_driver::NetworkHandle
  (:init
   (&key
    ((:type __type) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; uint32 _type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _type
       (write-long _type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::NetworkHandle :md5sum-) "fddb7a9f6752fdb043992b8a34032ae9")
(setf (get kortex_driver::NetworkHandle :datatype-) "kortex_driver/NetworkHandle")
(setf (get kortex_driver::NetworkHandle :definition-)
      "
uint32 type
")



(provide :kortex_driver/NetworkHandle "fddb7a9f6752fdb043992b8a34032ae9")


