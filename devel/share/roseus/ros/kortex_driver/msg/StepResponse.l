;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::StepResponse)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'StepResponse (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::STEPRESPONSE")
  (make-package "KORTEX_DRIVER::STEPRESPONSE"))

(in-package "ROS")
;;//! \htmlinclude StepResponse.msg.html


(defclass kortex_driver::StepResponse
  :super ros::object
  :slots (_loop_selection _amplitude _step_delay _duration ))

(defmethod kortex_driver::StepResponse
  (:init
   (&key
    ((:loop_selection __loop_selection) 0)
    ((:amplitude __amplitude) 0.0)
    ((:step_delay __step_delay) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _loop_selection (round __loop_selection))
   (setq _amplitude (float __amplitude))
   (setq _step_delay (float __step_delay))
   (setq _duration (float __duration))
   self)
  (:loop_selection
   (&optional __loop_selection)
   (if __loop_selection (setq _loop_selection __loop_selection)) _loop_selection)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:step_delay
   (&optional __step_delay)
   (if __step_delay (setq _step_delay __step_delay)) _step_delay)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _loop_selection
    4
    ;; float32 _amplitude
    4
    ;; float32 _step_delay
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
     ;; float32 _amplitude
       (sys::poke _amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _step_delay
       (sys::poke _step_delay (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _loop_selection
     (setq _loop_selection (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _amplitude
     (setq _amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _step_delay
     (setq _step_delay (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::StepResponse :md5sum-) "e9ae6749b10a5d88fae7f1a1e8639d93")
(setf (get kortex_driver::StepResponse :datatype-) "kortex_driver/StepResponse")
(setf (get kortex_driver::StepResponse :definition-)
      "
uint32 loop_selection
float32 amplitude
float32 step_delay
float32 duration
")



(provide :kortex_driver/StepResponse "e9ae6749b10a5d88fae7f1a1e8639d93")


