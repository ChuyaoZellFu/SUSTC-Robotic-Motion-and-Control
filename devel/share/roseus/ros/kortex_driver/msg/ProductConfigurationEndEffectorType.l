;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ProductConfigurationEndEffectorType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ProductConfigurationEndEffectorType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PRODUCTCONFIGURATIONENDEFFECTORTYPE")
  (make-package "KORTEX_DRIVER::PRODUCTCONFIGURATIONENDEFFECTORTYPE"))

(in-package "ROS")
;;//! \htmlinclude ProductConfigurationEndEffectorType.msg.html


(defclass kortex_driver::ProductConfigurationEndEffectorType
  :super ros::object
  :slots (_end_effector_type ))

(defmethod kortex_driver::ProductConfigurationEndEffectorType
  (:init
   (&key
    ((:end_effector_type __end_effector_type) 0)
    )
   (send-super :init)
   (setq _end_effector_type (round __end_effector_type))
   self)
  (:end_effector_type
   (&optional __end_effector_type)
   (if __end_effector_type (setq _end_effector_type __end_effector_type)) _end_effector_type)
  (:serialization-length
   ()
   (+
    ;; uint32 _end_effector_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _end_effector_type
       (write-long _end_effector_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _end_effector_type
     (setq _end_effector_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ProductConfigurationEndEffectorType :md5sum-) "471e42663382672f68e1e5c83df5fb95")
(setf (get kortex_driver::ProductConfigurationEndEffectorType :datatype-) "kortex_driver/ProductConfigurationEndEffectorType")
(setf (get kortex_driver::ProductConfigurationEndEffectorType :definition-)
      "
uint32 end_effector_type
")



(provide :kortex_driver/ProductConfigurationEndEffectorType "471e42663382672f68e1e5c83df5fb95")


