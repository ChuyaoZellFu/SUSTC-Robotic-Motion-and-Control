;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::EncoderDerivativeParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'EncoderDerivativeParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ENCODERDERIVATIVEPARAMETERS")
  (make-package "KORTEX_DRIVER::ENCODERDERIVATIVEPARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude EncoderDerivativeParameters.msg.html


(defclass kortex_driver::EncoderDerivativeParameters
  :super ros::object
  :slots (_max_window_width _min_angle ))

(defmethod kortex_driver::EncoderDerivativeParameters
  (:init
   (&key
    ((:max_window_width __max_window_width) 0)
    ((:min_angle __min_angle) 0.0)
    )
   (send-super :init)
   (setq _max_window_width (round __max_window_width))
   (setq _min_angle (float __min_angle))
   self)
  (:max_window_width
   (&optional __max_window_width)
   (if __max_window_width (setq _max_window_width __max_window_width)) _max_window_width)
  (:min_angle
   (&optional __min_angle)
   (if __min_angle (setq _min_angle __min_angle)) _min_angle)
  (:serialization-length
   ()
   (+
    ;; uint32 _max_window_width
    4
    ;; float32 _min_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _max_window_width
       (write-long _max_window_width s)
     ;; float32 _min_angle
       (sys::poke _min_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _max_window_width
     (setq _max_window_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _min_angle
     (setq _min_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::EncoderDerivativeParameters :md5sum-) "4e0a5e3a584720a2550104b5e1faa434")
(setf (get kortex_driver::EncoderDerivativeParameters :datatype-) "kortex_driver/EncoderDerivativeParameters")
(setf (get kortex_driver::EncoderDerivativeParameters :definition-)
      "
uint32 max_window_width
float32 min_angle
")



(provide :kortex_driver/EncoderDerivativeParameters "4e0a5e3a584720a2550104b5e1faa434")


