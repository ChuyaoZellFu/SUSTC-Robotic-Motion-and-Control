;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianLimitation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianLimitation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANLIMITATION")
  (make-package "KORTEX_DRIVER::CARTESIANLIMITATION"))

(in-package "ROS")
;;//! \htmlinclude CartesianLimitation.msg.html


(defclass kortex_driver::CartesianLimitation
  :super ros::object
  :slots (_type _translation _orientation ))

(defmethod kortex_driver::CartesianLimitation
  (:init
   (&key
    ((:type __type) 0)
    ((:translation __translation) 0.0)
    ((:orientation __orientation) 0.0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _translation (float __translation))
   (setq _orientation (float __orientation))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:translation
   (&optional __translation)
   (if __translation (setq _translation __translation)) _translation)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:serialization-length
   ()
   (+
    ;; uint32 _type
    4
    ;; float32 _translation
    4
    ;; float32 _orientation
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _type
       (write-long _type s)
     ;; float32 _translation
       (sys::poke _translation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientation
       (sys::poke _orientation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _translation
     (setq _translation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientation
     (setq _orientation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CartesianLimitation :md5sum-) "2764a5779bb20a3b4f16f3cce1cf0f4c")
(setf (get kortex_driver::CartesianLimitation :datatype-) "kortex_driver/CartesianLimitation")
(setf (get kortex_driver::CartesianLimitation :definition-)
      "
uint32 type
float32 translation
float32 orientation
")



(provide :kortex_driver/CartesianLimitation "2764a5779bb20a3b4f16f3cce1cf0f4c")


