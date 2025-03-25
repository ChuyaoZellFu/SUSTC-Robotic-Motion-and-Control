;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::TranslationVector)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'TranslationVector (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRANSLATIONVECTOR")
  (make-package "KORTEX_DRIVER::TRANSLATIONVECTOR"))

(in-package "ROS")
;;//! \htmlinclude TranslationVector.msg.html


(defclass kortex_driver::TranslationVector
  :super ros::object
  :slots (_t_x _t_y _t_z ))

(defmethod kortex_driver::TranslationVector
  (:init
   (&key
    ((:t_x __t_x) 0.0)
    ((:t_y __t_y) 0.0)
    ((:t_z __t_z) 0.0)
    )
   (send-super :init)
   (setq _t_x (float __t_x))
   (setq _t_y (float __t_y))
   (setq _t_z (float __t_z))
   self)
  (:t_x
   (&optional __t_x)
   (if __t_x (setq _t_x __t_x)) _t_x)
  (:t_y
   (&optional __t_y)
   (if __t_y (setq _t_y __t_y)) _t_y)
  (:t_z
   (&optional __t_z)
   (if __t_z (setq _t_z __t_z)) _t_z)
  (:serialization-length
   ()
   (+
    ;; float32 _t_x
    4
    ;; float32 _t_y
    4
    ;; float32 _t_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _t_x
       (sys::poke _t_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _t_y
       (sys::poke _t_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _t_z
       (sys::poke _t_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _t_x
     (setq _t_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _t_y
     (setq _t_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _t_z
     (setq _t_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::TranslationVector :md5sum-) "1c59f12cff6754b509028be4dd466647")
(setf (get kortex_driver::TranslationVector :datatype-) "kortex_driver/TranslationVector")
(setf (get kortex_driver::TranslationVector :definition-)
      "
float32 t_x
float32 t_y
float32 t_z
")



(provide :kortex_driver/TranslationVector "1c59f12cff6754b509028be4dd466647")


