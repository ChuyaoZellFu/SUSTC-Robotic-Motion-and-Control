;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GripperRequest)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GripperRequest (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPERREQUEST")
  (make-package "KORTEX_DRIVER::GRIPPERREQUEST"))

(in-package "ROS")
;;//! \htmlinclude GripperRequest.msg.html


(defclass kortex_driver::GripperRequest
  :super ros::object
  :slots (_mode ))

(defmethod kortex_driver::GripperRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; uint32 _mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _mode
       (write-long _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::GripperRequest :md5sum-) "890d2fbf5b6e45074133a9b18a5f56b6")
(setf (get kortex_driver::GripperRequest :datatype-) "kortex_driver/GripperRequest")
(setf (get kortex_driver::GripperRequest :definition-)
      "
uint32 mode
")



(provide :kortex_driver/GripperRequest "890d2fbf5b6e45074133a9b18a5f56b6")


