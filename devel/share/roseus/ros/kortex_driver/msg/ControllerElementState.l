;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerElementState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerElementState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERELEMENTSTATE")
  (make-package "KORTEX_DRIVER::CONTROLLERELEMENTSTATE"))

(in-package "ROS")
;;//! \htmlinclude ControllerElementState.msg.html


(defclass kortex_driver::ControllerElementState
  :super ros::object
  :slots (_handle _event_type _axis_value ))

(defmethod kortex_driver::ControllerElementState
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::ControllerElementHandle :init))
    ((:event_type __event_type) 0)
    ((:axis_value __axis_value) 0.0)
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _event_type (round __event_type))
   (setq _axis_value (float __axis_value))
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:event_type
   (&optional __event_type)
   (if __event_type (setq _event_type __event_type)) _event_type)
  (:axis_value
   (&optional __axis_value)
   (if __axis_value (setq _axis_value __axis_value)) _axis_value)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerElementHandle _handle
    (send _handle :serialization-length)
    ;; uint32 _event_type
    4
    ;; float32 _axis_value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerElementHandle _handle
       (send _handle :serialize s)
     ;; uint32 _event_type
       (write-long _event_type s)
     ;; float32 _axis_value
       (sys::poke _axis_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerElementHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; uint32 _event_type
     (setq _event_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _axis_value
     (setq _axis_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ControllerElementState :md5sum-) "522b5634710ce23afc015e933c822f1e")
(setf (get kortex_driver::ControllerElementState :datatype-) "kortex_driver/ControllerElementState")
(setf (get kortex_driver::ControllerElementState :definition-)
      "
ControllerElementHandle handle
uint32 event_type
float32 axis_value
================================================================================
MSG: kortex_driver/ControllerElementHandle

ControllerHandle controller_handle
ControllerElementHandle_identifier oneof_identifier
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/ControllerElementHandle_identifier

uint32[] button
uint32[] axis
")



(provide :kortex_driver/ControllerElementState "522b5634710ce23afc015e933c822f1e")


