;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MapEvent_events)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MapEvent_events (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPEVENT_EVENTS")
  (make-package "KORTEX_DRIVER::MAPEVENT_EVENTS"))

(in-package "ROS")
;;//! \htmlinclude MapEvent_events.msg.html


(defclass kortex_driver::MapEvent_events
  :super ros::object
  :slots (_safety_event _gpio_event _controller_event ))

(defmethod kortex_driver::MapEvent_events
  (:init
   (&key
    ((:safety_event __safety_event) ())
    ((:gpio_event __gpio_event) ())
    ((:controller_event __controller_event) ())
    )
   (send-super :init)
   (setq _safety_event __safety_event)
   (setq _gpio_event __gpio_event)
   (setq _controller_event __controller_event)
   self)
  (:safety_event
   (&rest __safety_event)
   (if (keywordp (car __safety_event))
       (send* _safety_event __safety_event)
     (progn
       (if __safety_event (setq _safety_event (car __safety_event)))
       _safety_event)))
  (:gpio_event
   (&rest __gpio_event)
   (if (keywordp (car __gpio_event))
       (send* _gpio_event __gpio_event)
     (progn
       (if __gpio_event (setq _gpio_event (car __gpio_event)))
       _gpio_event)))
  (:controller_event
   (&rest __controller_event)
   (if (keywordp (car __controller_event))
       (send* _controller_event __controller_event)
     (progn
       (if __controller_event (setq _controller_event (car __controller_event)))
       _controller_event)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyEvent[] _safety_event
    (apply #'+ (send-all _safety_event :serialization-length)) 4
    ;; kortex_driver/GpioEvent[] _gpio_event
    (apply #'+ (send-all _gpio_event :serialization-length)) 4
    ;; kortex_driver/ControllerEvent[] _controller_event
    (apply #'+ (send-all _controller_event :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyEvent[] _safety_event
     (write-long (length _safety_event) s)
     (dolist (elem _safety_event)
       (send elem :serialize s)
       )
     ;; kortex_driver/GpioEvent[] _gpio_event
     (write-long (length _gpio_event) s)
     (dolist (elem _gpio_event)
       (send elem :serialize s)
       )
     ;; kortex_driver/ControllerEvent[] _controller_event
     (write-long (length _controller_event) s)
     (dolist (elem _controller_event)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyEvent[] _safety_event
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _safety_event (let (r) (dotimes (i n) (push (instance kortex_driver::SafetyEvent :init) r)) r))
     (dolist (elem- _safety_event)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/GpioEvent[] _gpio_event
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gpio_event (let (r) (dotimes (i n) (push (instance kortex_driver::GpioEvent :init) r)) r))
     (dolist (elem- _gpio_event)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ControllerEvent[] _controller_event
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controller_event (let (r) (dotimes (i n) (push (instance kortex_driver::ControllerEvent :init) r)) r))
     (dolist (elem- _controller_event)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::MapEvent_events :md5sum-) "040caa44c991691249bac9565abbf70c")
(setf (get kortex_driver::MapEvent_events :datatype-) "kortex_driver/MapEvent_events")
(setf (get kortex_driver::MapEvent_events :definition-)
      "
SafetyEvent[] safety_event
GpioEvent[] gpio_event
ControllerEvent[] controller_event
================================================================================
MSG: kortex_driver/SafetyEvent

SafetyHandle safety_handle
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
================================================================================
MSG: kortex_driver/GpioEvent

uint32 input_type
uint32 behavior
uint32 input_identifier
================================================================================
MSG: kortex_driver/ControllerEvent

uint32 input_type
uint32 behavior
uint32 input_identifier
")



(provide :kortex_driver/MapEvent_events "040caa44c991691249bac9565abbf70c")


