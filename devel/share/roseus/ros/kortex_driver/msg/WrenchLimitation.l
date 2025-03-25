;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::WrenchLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'WrenchLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::WRENCHLIMITATION")
  (make-package "KORTEX_DRIVER::WRENCHLIMITATION"))

(in-package "ROS")
;;//! \htmlinclude WrenchLimitation.msg.html


(defclass kortex_driver::WrenchLimitation
  :super ros::object
  :slots (_force _torque ))

(defmethod kortex_driver::WrenchLimitation
  (:init
   (&key
    ((:force __force) 0.0)
    ((:torque __torque) 0.0)
    )
   (send-super :init)
   (setq _force (float __force))
   (setq _torque (float __torque))
   self)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:serialization-length
   ()
   (+
    ;; float32 _force
    4
    ;; float32 _torque
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _force
       (sys::poke _force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _torque
       (sys::poke _torque (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _force
     (setq _force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _torque
     (setq _torque (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::WrenchLimitation :md5sum-) "2afe6a3b4cba4f0de2ad91545187e9d1")
(setf (get kortex_driver::WrenchLimitation :datatype-) "kortex_driver/WrenchLimitation")
(setf (get kortex_driver::WrenchLimitation :definition-)
      "
float32 force
float32 torque
")



(provide :kortex_driver/WrenchLimitation "2afe6a3b4cba4f0de2ad91545187e9d1")


