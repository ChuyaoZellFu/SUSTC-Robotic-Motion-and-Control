;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControlLoopParameters)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControlLoopParameters (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLOOPPARAMETERS")
  (make-package "KORTEX_DRIVER::CONTROLLOOPPARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude ControlLoopParameters.msg.html


(defclass kortex_driver::ControlLoopParameters
  :super ros::object
  :slots (_loop_selection _error_saturation _output_saturation _kAz _kBz _error_dead_band ))

(defmethod kortex_driver::ControlLoopParameters
  (:init
   (&key
    ((:loop_selection __loop_selection) 0)
    ((:error_saturation __error_saturation) 0.0)
    ((:output_saturation __output_saturation) 0.0)
    ((:kAz __kAz) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:kBz __kBz) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:error_dead_band __error_dead_band) 0.0)
    )
   (send-super :init)
   (setq _loop_selection (round __loop_selection))
   (setq _error_saturation (float __error_saturation))
   (setq _output_saturation (float __output_saturation))
   (setq _kAz __kAz)
   (setq _kBz __kBz)
   (setq _error_dead_band (float __error_dead_band))
   self)
  (:loop_selection
   (&optional __loop_selection)
   (if __loop_selection (setq _loop_selection __loop_selection)) _loop_selection)
  (:error_saturation
   (&optional __error_saturation)
   (if __error_saturation (setq _error_saturation __error_saturation)) _error_saturation)
  (:output_saturation
   (&optional __output_saturation)
   (if __output_saturation (setq _output_saturation __output_saturation)) _output_saturation)
  (:kAz
   (&optional __kAz)
   (if __kAz (setq _kAz __kAz)) _kAz)
  (:kBz
   (&optional __kBz)
   (if __kBz (setq _kBz __kBz)) _kBz)
  (:error_dead_band
   (&optional __error_dead_band)
   (if __error_dead_band (setq _error_dead_band __error_dead_band)) _error_dead_band)
  (:serialization-length
   ()
   (+
    ;; uint32 _loop_selection
    4
    ;; float32 _error_saturation
    4
    ;; float32 _output_saturation
    4
    ;; float32[] _kAz
    (* 4    (length _kAz)) 4
    ;; float32[] _kBz
    (* 4    (length _kBz)) 4
    ;; float32 _error_dead_band
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _loop_selection
       (write-long _loop_selection s)
     ;; float32 _error_saturation
       (sys::poke _error_saturation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _output_saturation
       (sys::poke _output_saturation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _kAz
     (write-long (length _kAz) s)
     (dotimes (i (length _kAz))
       (sys::poke (elt _kAz i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _kBz
     (write-long (length _kBz) s)
     (dotimes (i (length _kBz))
       (sys::poke (elt _kBz i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _error_dead_band
       (sys::poke _error_dead_band (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _loop_selection
     (setq _loop_selection (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _error_saturation
     (setq _error_saturation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _output_saturation
     (setq _output_saturation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _kAz
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kAz (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _kAz i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _kBz
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kBz (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _kBz i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _error_dead_band
     (setq _error_dead_band (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ControlLoopParameters :md5sum-) "309e14eef078d5bea86d94317d3d0e04")
(setf (get kortex_driver::ControlLoopParameters :datatype-) "kortex_driver/ControlLoopParameters")
(setf (get kortex_driver::ControlLoopParameters :definition-)
      "
uint32 loop_selection
float32 error_saturation
float32 output_saturation
float32[] kAz
float32[] kBz
float32 error_dead_band
")



(provide :kortex_driver/ControlLoopParameters "309e14eef078d5bea86d94317d3d0e04")


