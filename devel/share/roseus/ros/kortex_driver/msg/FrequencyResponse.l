;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FrequencyResponse)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FrequencyResponse (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FREQUENCYRESPONSE")
  (make-package "KORTEX_DRIVER::FREQUENCYRESPONSE"))

(in-package "ROS")
;;//! \htmlinclude FrequencyResponse.msg.html


(defclass kortex_driver::FrequencyResponse
  :super ros::object
  :slots (_loop_selection _min_frequency _max_frequency _amplitude _duration ))

(defmethod kortex_driver::FrequencyResponse
  (:init
   (&key
    ((:loop_selection __loop_selection) 0)
    ((:min_frequency __min_frequency) 0.0)
    ((:max_frequency __max_frequency) 0.0)
    ((:amplitude __amplitude) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _loop_selection (round __loop_selection))
   (setq _min_frequency (float __min_frequency))
   (setq _max_frequency (float __max_frequency))
   (setq _amplitude (float __amplitude))
   (setq _duration (float __duration))
   self)
  (:loop_selection
   (&optional __loop_selection)
   (if __loop_selection (setq _loop_selection __loop_selection)) _loop_selection)
  (:min_frequency
   (&optional __min_frequency)
   (if __min_frequency (setq _min_frequency __min_frequency)) _min_frequency)
  (:max_frequency
   (&optional __max_frequency)
   (if __max_frequency (setq _max_frequency __max_frequency)) _max_frequency)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint32 _loop_selection
    4
    ;; float32 _min_frequency
    4
    ;; float32 _max_frequency
    4
    ;; float32 _amplitude
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
     ;; float32 _min_frequency
       (sys::poke _min_frequency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_frequency
       (sys::poke _max_frequency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _amplitude
       (sys::poke _amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _loop_selection
     (setq _loop_selection (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _min_frequency
     (setq _min_frequency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_frequency
     (setq _max_frequency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _amplitude
     (setq _amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::FrequencyResponse :md5sum-) "86ed7a41c264d9e5ff7a52aa97222d1e")
(setf (get kortex_driver::FrequencyResponse :datatype-) "kortex_driver/FrequencyResponse")
(setf (get kortex_driver::FrequencyResponse :definition-)
      "
uint32 loop_selection
float32 min_frequency
float32 max_frequency
float32 amplitude
float32 duration
")



(provide :kortex_driver/FrequencyResponse "86ed7a41c264d9e5ff7a52aa97222d1e")


