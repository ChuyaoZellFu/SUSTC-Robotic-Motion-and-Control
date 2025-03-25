;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RequestedActionType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RequestedActionType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::REQUESTEDACTIONTYPE")
  (make-package "KORTEX_DRIVER::REQUESTEDACTIONTYPE"))

(in-package "ROS")
;;//! \htmlinclude RequestedActionType.msg.html


(defclass kortex_driver::RequestedActionType
  :super ros::object
  :slots (_action_type ))

(defmethod kortex_driver::RequestedActionType
  (:init
   (&key
    ((:action_type __action_type) 0)
    )
   (send-super :init)
   (setq _action_type (round __action_type))
   self)
  (:action_type
   (&optional __action_type)
   (if __action_type (setq _action_type __action_type)) _action_type)
  (:serialization-length
   ()
   (+
    ;; uint32 _action_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _action_type
       (write-long _action_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _action_type
     (setq _action_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::RequestedActionType :md5sum-) "38655324c3c40f2a5f27841a581c389b")
(setf (get kortex_driver::RequestedActionType :datatype-) "kortex_driver/RequestedActionType")
(setf (get kortex_driver::RequestedActionType :definition-)
      "
uint32 action_type
")



(provide :kortex_driver/RequestedActionType "38655324c3c40f2a5f27841a581c389b")


