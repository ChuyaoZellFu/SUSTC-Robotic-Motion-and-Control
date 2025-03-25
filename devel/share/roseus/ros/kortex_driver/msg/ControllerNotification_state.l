;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerNotification_state)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerNotification_state (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERNOTIFICATION_STATE")
  (make-package "KORTEX_DRIVER::CONTROLLERNOTIFICATION_STATE"))

(in-package "ROS")
;;//! \htmlinclude ControllerNotification_state.msg.html


(defclass kortex_driver::ControllerNotification_state
  :super ros::object
  :slots (_controller_state _controller_element ))

(defmethod kortex_driver::ControllerNotification_state
  (:init
   (&key
    ((:controller_state __controller_state) ())
    ((:controller_element __controller_element) ())
    )
   (send-super :init)
   (setq _controller_state __controller_state)
   (setq _controller_element __controller_element)
   self)
  (:controller_state
   (&rest __controller_state)
   (if (keywordp (car __controller_state))
       (send* _controller_state __controller_state)
     (progn
       (if __controller_state (setq _controller_state (car __controller_state)))
       _controller_state)))
  (:controller_element
   (&rest __controller_element)
   (if (keywordp (car __controller_element))
       (send* _controller_element __controller_element)
     (progn
       (if __controller_element (setq _controller_element (car __controller_element)))
       _controller_element)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerState[] _controller_state
    (apply #'+ (send-all _controller_state :serialization-length)) 4
    ;; kortex_driver/ControllerElementState[] _controller_element
    (apply #'+ (send-all _controller_element :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerState[] _controller_state
     (write-long (length _controller_state) s)
     (dolist (elem _controller_state)
       (send elem :serialize s)
       )
     ;; kortex_driver/ControllerElementState[] _controller_element
     (write-long (length _controller_element) s)
     (dolist (elem _controller_element)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerState[] _controller_state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controller_state (let (r) (dotimes (i n) (push (instance kortex_driver::ControllerState :init) r)) r))
     (dolist (elem- _controller_state)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/ControllerElementState[] _controller_element
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controller_element (let (r) (dotimes (i n) (push (instance kortex_driver::ControllerElementState :init) r)) r))
     (dolist (elem- _controller_element)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerNotification_state :md5sum-) "f624e32135515547aa37c46203c9be9e")
(setf (get kortex_driver::ControllerNotification_state :datatype-) "kortex_driver/ControllerNotification_state")
(setf (get kortex_driver::ControllerNotification_state :definition-)
      "
ControllerState[] controller_state
ControllerElementState[] controller_element
================================================================================
MSG: kortex_driver/ControllerState

ControllerHandle handle
uint32 event_type
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/ControllerElementState

ControllerElementHandle handle
uint32 event_type
float32 axis_value
================================================================================
MSG: kortex_driver/ControllerElementHandle

ControllerHandle controller_handle
ControllerElementHandle_identifier oneof_identifier
================================================================================
MSG: kortex_driver/ControllerElementHandle_identifier

uint32[] button
uint32[] axis
")



(provide :kortex_driver/ControllerNotification_state "f624e32135515547aa37c46203c9be9e")


