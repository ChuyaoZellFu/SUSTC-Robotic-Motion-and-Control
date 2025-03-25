;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Snapshot)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Snapshot (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SNAPSHOT")
  (make-package "KORTEX_DRIVER::SNAPSHOT"))

(in-package "ROS")
;;//! \htmlinclude Snapshot.msg.html


(defclass kortex_driver::Snapshot
  :super ros::object
  :slots (_snapshot_type ))

(defmethod kortex_driver::Snapshot
  (:init
   (&key
    ((:snapshot_type __snapshot_type) 0)
    )
   (send-super :init)
   (setq _snapshot_type (round __snapshot_type))
   self)
  (:snapshot_type
   (&optional __snapshot_type)
   (if __snapshot_type (setq _snapshot_type __snapshot_type)) _snapshot_type)
  (:serialization-length
   ()
   (+
    ;; uint32 _snapshot_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _snapshot_type
       (write-long _snapshot_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _snapshot_type
     (setq _snapshot_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::Snapshot :md5sum-) "d1bcc273f3d53a1ef917f40e2afafc1e")
(setf (get kortex_driver::Snapshot :datatype-) "kortex_driver/Snapshot")
(setf (get kortex_driver::Snapshot :definition-)
      "
uint32 snapshot_type
")



(provide :kortex_driver/Snapshot "d1bcc273f3d53a1ef917f40e2afafc1e")


