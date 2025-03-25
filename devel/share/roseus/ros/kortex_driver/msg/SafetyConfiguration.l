;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYCONFIGURATION")
  (make-package "KORTEX_DRIVER::SAFETYCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude SafetyConfiguration.msg.html


(defclass kortex_driver::SafetyConfiguration
  :super ros::object
  :slots (_handle _error_threshold _warning_threshold _enable ))

(defmethod kortex_driver::SafetyConfiguration
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::SafetyHandle :init))
    ((:error_threshold __error_threshold) 0.0)
    ((:warning_threshold __warning_threshold) 0.0)
    ((:enable __enable) (instance kortex_driver::SafetyEnable :init))
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _error_threshold (float __error_threshold))
   (setq _warning_threshold (float __warning_threshold))
   (setq _enable __enable)
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:error_threshold
   (&optional __error_threshold)
   (if __error_threshold (setq _error_threshold __error_threshold)) _error_threshold)
  (:warning_threshold
   (&optional __warning_threshold)
   (if __warning_threshold (setq _warning_threshold __warning_threshold)) _warning_threshold)
  (:enable
   (&rest __enable)
   (if (keywordp (car __enable))
       (send* _enable __enable)
     (progn
       (if __enable (setq _enable (car __enable)))
       _enable)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyHandle _handle
    (send _handle :serialization-length)
    ;; float32 _error_threshold
    4
    ;; float32 _warning_threshold
    4
    ;; kortex_driver/SafetyEnable _enable
    (send _enable :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _handle
       (send _handle :serialize s)
     ;; float32 _error_threshold
       (sys::poke _error_threshold (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _warning_threshold
       (sys::poke _warning_threshold (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; kortex_driver/SafetyEnable _enable
       (send _enable :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; float32 _error_threshold
     (setq _error_threshold (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _warning_threshold
     (setq _warning_threshold (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; kortex_driver/SafetyEnable _enable
     (send _enable :deserialize buf ptr-) (incf ptr- (send _enable :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::SafetyConfiguration :md5sum-) "42160db695be444fee95ed355f4763b8")
(setf (get kortex_driver::SafetyConfiguration :datatype-) "kortex_driver/SafetyConfiguration")
(setf (get kortex_driver::SafetyConfiguration :definition-)
      "
SafetyHandle handle
float32 error_threshold
float32 warning_threshold
SafetyEnable enable
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
bool enable
")



(provide :kortex_driver/SafetyConfiguration "42160db695be444fee95ed355f4763b8")


