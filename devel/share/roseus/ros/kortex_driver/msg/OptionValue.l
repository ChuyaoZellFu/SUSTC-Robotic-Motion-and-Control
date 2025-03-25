;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OptionValue)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OptionValue (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPTIONVALUE")
  (make-package "KORTEX_DRIVER::OPTIONVALUE"))

(in-package "ROS")
;;//! \htmlinclude OptionValue.msg.html


(defclass kortex_driver::OptionValue
  :super ros::object
  :slots (_sensor _option _value ))

(defmethod kortex_driver::OptionValue
  (:init
   (&key
    ((:sensor __sensor) 0)
    ((:option __option) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   (setq _option (round __option))
   (setq _value (float __value))
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:option
   (&optional __option)
   (if __option (setq _option __option)) _option)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ;; uint32 _option
    4
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sensor
       (write-long _sensor s)
     ;; uint32 _option
       (write-long _option s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _option
     (setq _option (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::OptionValue :md5sum-) "fa7cb8d520a3aad3885966319f33ad46")
(setf (get kortex_driver::OptionValue :datatype-) "kortex_driver/OptionValue")
(setf (get kortex_driver::OptionValue :definition-)
      "
uint32 sensor
uint32 option
float32 value
")



(provide :kortex_driver/OptionValue "fa7cb8d520a3aad3885966319f33ad46")


