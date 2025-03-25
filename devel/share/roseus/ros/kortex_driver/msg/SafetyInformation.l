;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYINFORMATION")
  (make-package "KORTEX_DRIVER::SAFETYINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude SafetyInformation.msg.html


(defclass kortex_driver::SafetyInformation
  :super ros::object
  :slots (_handle _can_change_safety_state _has_warning_threshold _has_error_threshold _limit_type _default_warning_threshold _default_error_threshold _upper_hard_limit _lower_hard_limit _status _unit ))

(defmethod kortex_driver::SafetyInformation
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::SafetyHandle :init))
    ((:can_change_safety_state __can_change_safety_state) nil)
    ((:has_warning_threshold __has_warning_threshold) nil)
    ((:has_error_threshold __has_error_threshold) nil)
    ((:limit_type __limit_type) 0)
    ((:default_warning_threshold __default_warning_threshold) 0.0)
    ((:default_error_threshold __default_error_threshold) 0.0)
    ((:upper_hard_limit __upper_hard_limit) 0.0)
    ((:lower_hard_limit __lower_hard_limit) 0.0)
    ((:status __status) 0)
    ((:unit __unit) 0)
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _can_change_safety_state __can_change_safety_state)
   (setq _has_warning_threshold __has_warning_threshold)
   (setq _has_error_threshold __has_error_threshold)
   (setq _limit_type (round __limit_type))
   (setq _default_warning_threshold (float __default_warning_threshold))
   (setq _default_error_threshold (float __default_error_threshold))
   (setq _upper_hard_limit (float __upper_hard_limit))
   (setq _lower_hard_limit (float __lower_hard_limit))
   (setq _status (round __status))
   (setq _unit (round __unit))
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:can_change_safety_state
   (&optional (__can_change_safety_state :null))
   (if (not (eq __can_change_safety_state :null)) (setq _can_change_safety_state __can_change_safety_state)) _can_change_safety_state)
  (:has_warning_threshold
   (&optional (__has_warning_threshold :null))
   (if (not (eq __has_warning_threshold :null)) (setq _has_warning_threshold __has_warning_threshold)) _has_warning_threshold)
  (:has_error_threshold
   (&optional (__has_error_threshold :null))
   (if (not (eq __has_error_threshold :null)) (setq _has_error_threshold __has_error_threshold)) _has_error_threshold)
  (:limit_type
   (&optional __limit_type)
   (if __limit_type (setq _limit_type __limit_type)) _limit_type)
  (:default_warning_threshold
   (&optional __default_warning_threshold)
   (if __default_warning_threshold (setq _default_warning_threshold __default_warning_threshold)) _default_warning_threshold)
  (:default_error_threshold
   (&optional __default_error_threshold)
   (if __default_error_threshold (setq _default_error_threshold __default_error_threshold)) _default_error_threshold)
  (:upper_hard_limit
   (&optional __upper_hard_limit)
   (if __upper_hard_limit (setq _upper_hard_limit __upper_hard_limit)) _upper_hard_limit)
  (:lower_hard_limit
   (&optional __lower_hard_limit)
   (if __lower_hard_limit (setq _lower_hard_limit __lower_hard_limit)) _lower_hard_limit)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:unit
   (&optional __unit)
   (if __unit (setq _unit __unit)) _unit)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyHandle _handle
    (send _handle :serialization-length)
    ;; bool _can_change_safety_state
    1
    ;; bool _has_warning_threshold
    1
    ;; bool _has_error_threshold
    1
    ;; uint32 _limit_type
    4
    ;; float32 _default_warning_threshold
    4
    ;; float32 _default_error_threshold
    4
    ;; float32 _upper_hard_limit
    4
    ;; float32 _lower_hard_limit
    4
    ;; uint32 _status
    4
    ;; uint32 _unit
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _handle
       (send _handle :serialize s)
     ;; bool _can_change_safety_state
       (if _can_change_safety_state (write-byte -1 s) (write-byte 0 s))
     ;; bool _has_warning_threshold
       (if _has_warning_threshold (write-byte -1 s) (write-byte 0 s))
     ;; bool _has_error_threshold
       (if _has_error_threshold (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _limit_type
       (write-long _limit_type s)
     ;; float32 _default_warning_threshold
       (sys::poke _default_warning_threshold (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _default_error_threshold
       (sys::poke _default_error_threshold (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _upper_hard_limit
       (sys::poke _upper_hard_limit (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lower_hard_limit
       (sys::poke _lower_hard_limit (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _status
       (write-long _status s)
     ;; uint32 _unit
       (write-long _unit s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; bool _can_change_safety_state
     (setq _can_change_safety_state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _has_warning_threshold
     (setq _has_warning_threshold (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _has_error_threshold
     (setq _has_error_threshold (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _limit_type
     (setq _limit_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _default_warning_threshold
     (setq _default_warning_threshold (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _default_error_threshold
     (setq _default_error_threshold (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _upper_hard_limit
     (setq _upper_hard_limit (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lower_hard_limit
     (setq _lower_hard_limit (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _unit
     (setq _unit (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SafetyInformation :md5sum-) "e8597ef9acfa23c653020b88d86d8b2f")
(setf (get kortex_driver::SafetyInformation :datatype-) "kortex_driver/SafetyInformation")
(setf (get kortex_driver::SafetyInformation :definition-)
      "
SafetyHandle handle
bool can_change_safety_state
bool has_warning_threshold
bool has_error_threshold
uint32 limit_type
float32 default_warning_threshold
float32 default_error_threshold
float32 upper_hard_limit
float32 lower_hard_limit
uint32 status
uint32 unit
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
")



(provide :kortex_driver/SafetyInformation "e8597ef9acfa23c653020b88d86d8b2f")


