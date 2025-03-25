;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OptionInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OptionInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPTIONINFORMATION")
  (make-package "KORTEX_DRIVER::OPTIONINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude OptionInformation.msg.html


(defclass kortex_driver::OptionInformation
  :super ros::object
  :slots (_sensor _option _supported _read_only _minimum _maximum _step _default_value ))

(defmethod kortex_driver::OptionInformation
  (:init
   (&key
    ((:sensor __sensor) 0)
    ((:option __option) 0)
    ((:supported __supported) nil)
    ((:read_only __read_only) nil)
    ((:minimum __minimum) 0.0)
    ((:maximum __maximum) 0.0)
    ((:step __step) 0.0)
    ((:default_value __default_value) 0.0)
    )
   (send-super :init)
   (setq _sensor (round __sensor))
   (setq _option (round __option))
   (setq _supported __supported)
   (setq _read_only __read_only)
   (setq _minimum (float __minimum))
   (setq _maximum (float __maximum))
   (setq _step (float __step))
   (setq _default_value (float __default_value))
   self)
  (:sensor
   (&optional __sensor)
   (if __sensor (setq _sensor __sensor)) _sensor)
  (:option
   (&optional __option)
   (if __option (setq _option __option)) _option)
  (:supported
   (&optional (__supported :null))
   (if (not (eq __supported :null)) (setq _supported __supported)) _supported)
  (:read_only
   (&optional (__read_only :null))
   (if (not (eq __read_only :null)) (setq _read_only __read_only)) _read_only)
  (:minimum
   (&optional __minimum)
   (if __minimum (setq _minimum __minimum)) _minimum)
  (:maximum
   (&optional __maximum)
   (if __maximum (setq _maximum __maximum)) _maximum)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:default_value
   (&optional __default_value)
   (if __default_value (setq _default_value __default_value)) _default_value)
  (:serialization-length
   ()
   (+
    ;; uint32 _sensor
    4
    ;; uint32 _option
    4
    ;; bool _supported
    1
    ;; bool _read_only
    1
    ;; float32 _minimum
    4
    ;; float32 _maximum
    4
    ;; float32 _step
    4
    ;; float32 _default_value
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
     ;; bool _supported
       (if _supported (write-byte -1 s) (write-byte 0 s))
     ;; bool _read_only
       (if _read_only (write-byte -1 s) (write-byte 0 s))
     ;; float32 _minimum
       (sys::poke _minimum (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _maximum
       (sys::poke _maximum (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _step
       (sys::poke _step (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _default_value
       (sys::poke _default_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sensor
     (setq _sensor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _option
     (setq _option (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _supported
     (setq _supported (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _read_only
     (setq _read_only (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _minimum
     (setq _minimum (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _maximum
     (setq _maximum (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _step
     (setq _step (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _default_value
     (setq _default_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::OptionInformation :md5sum-) "4a9212bde4d95812b702827b151ff2e7")
(setf (get kortex_driver::OptionInformation :datatype-) "kortex_driver/OptionInformation")
(setf (get kortex_driver::OptionInformation :definition-)
      "
uint32 sensor
uint32 option
bool supported
bool read_only
float32 minimum
float32 maximum
float32 step
float32 default_value
")



(provide :kortex_driver/OptionInformation "4a9212bde4d95812b702827b151ff2e7")


