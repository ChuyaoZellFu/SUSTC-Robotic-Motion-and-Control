;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RampResponse)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RampResponse (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RAMPRESPONSE")
  (make-package "KORTEX_DRIVER::RAMPRESPONSE"))

(in-package "ROS")
;;//! \htmlinclude RampResponse.msg.html


(defclass kortex_driver::RampResponse
  :super ros::object
  :slots (_loop_selection _slope _ramp_delay _duration ))

(defmethod kortex_driver::RampResponse
  (:init
   (&key
    ((:loop_selection __loop_selection) 0)
    ((:slope __slope) 0.0)
    ((:ramp_delay __ramp_delay) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _loop_selection (round __loop_selection))
   (setq _slope (float __slope))
   (setq _ramp_delay (float __ramp_delay))
   (setq _duration (float __duration))
   self)
  (:loop_selection
   (&optional __loop_selection)
   (if __loop_selection (setq _loop_selection __loop_selection)) _loop_selection)
  (:slope
   (&optional __slope)
   (if __slope (setq _slope __slope)) _slope)
  (:ramp_delay
   (&optional __ramp_delay)
   (if __ramp_delay (setq _ramp_delay __ramp_delay)) _ramp_delay)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _loop_selection
    4
    ;; float32 _slope
    4
    ;; float32 _ramp_delay
    4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _loop_selection
       (write-long _loop_selection s)
     ;; float32 _slope
       (sys::poke _slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ramp_delay
       (sys::poke _ramp_delay (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _loop_selection
     (setq _loop_selection (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _slope
     (setq _slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ramp_delay
     (setq _ramp_delay (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::RampResponse :md5sum-) "9e3f4edf9087b42bf3e2e7ff4ca2b2f9")
(setf (get kortex_driver::RampResponse :datatype-) "kortex_driver/RampResponse")
(setf (get kortex_driver::RampResponse :definition-)
      "
uint32 loop_selection
float32 slope
float32 ramp_delay
float32 duration
")



(provide :kortex_driver/RampResponse "9e3f4edf9087b42bf3e2e7ff4ca2b2f9")


