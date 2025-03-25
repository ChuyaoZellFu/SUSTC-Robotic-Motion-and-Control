;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::MapEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'MapEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MAPEVENT")
  (make-package "KORTEX_DRIVER::MAPEVENT"))

(in-package "ROS")
;;//! \htmlinclude MapEvent.msg.html


(defclass kortex_driver::MapEvent
  :super ros::object
  :slots (_name _oneof_events ))

(defmethod kortex_driver::MapEvent
  (:init
   (&key
    ((:name __name) "")
    ((:oneof_events __oneof_events) (instance kortex_driver::MapEvent_events :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _oneof_events __oneof_events)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:oneof_events
   (&rest __oneof_events)
   (if (keywordp (car __oneof_events))
       (send* _oneof_events __oneof_events)
     (progn
       (if __oneof_events (setq _oneof_events (car __oneof_events)))
       _oneof_events)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; kortex_driver/MapEvent_events _oneof_events
    (send _oneof_events :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; kortex_driver/MapEvent_events _oneof_events
       (send _oneof_events :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kortex_driver/MapEvent_events _oneof_events
     (send _oneof_events :deserialize buf ptr-) (incf ptr- (send _oneof_events :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::MapEvent :md5sum-) "c2332688b9b21902d710a4725da3d8c9")
(setf (get kortex_driver::MapEvent :datatype-) "kortex_driver/MapEvent")
(setf (get kortex_driver::MapEvent :definition-)
      "
string name
MapEvent_events oneof_events
================================================================================
MSG: kortex_driver/MapEvent_events

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



(provide :kortex_driver/MapEvent "c2332688b9b21902d710a4725da3d8c9")


