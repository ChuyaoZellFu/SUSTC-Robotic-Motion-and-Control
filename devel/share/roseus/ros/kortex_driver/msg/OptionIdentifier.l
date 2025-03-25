;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OptionIdentifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OptionIdentifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPTIONIDENTIFIER")
  (make-package "KORTEX_DRIVER::OPTIONIDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude OptionIdentifier.msg.html


(defclass kortex_driver::OptionIdentifier
  :super ros::object
  :slots (_sensor _option ))

(defmethod kortex_driver::OptionIdentifier
  (:init
   (&key
    ((:sensor __sensor) 0)
    ((:option __option) 0)
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   (setq _option (round __option))
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:option
   (&optional __option)
   (if __option (setq _option __option)) _option)
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ;; uint32 _option
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _option
     (setq _option (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::OptionIdentifier :md5sum-) "23d53d4ecb95c9409f8e27d3bfff4aca")
(setf (get kortex_driver::OptionIdentifier :datatype-) "kortex_driver/OptionIdentifier")
(setf (get kortex_driver::OptionIdentifier :definition-)
      "
uint32 sensor
uint32 option
")



(provide :kortex_driver/OptionIdentifier "23d53d4ecb95c9409f8e27d3bfff4aca")


