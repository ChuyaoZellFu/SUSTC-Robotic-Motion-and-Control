;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ActionExecutionState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ActionExecutionState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ACTIONEXECUTIONSTATE")
  (make-package "KORTEX_DRIVER::ACTIONEXECUTIONSTATE"))

(in-package "ROS")
;;//! \htmlinclude ActionExecutionState.msg.html


(defclass kortex_driver::ActionExecutionState
  :super ros::object
  :slots (_action_event _handle ))

(defmethod kortex_driver::ActionExecutionState
  (:init
   (&key
    ((:action_event __action_event) 0)
    ((:handle __handle) (instance kortex_driver::ActionHandle :init))
    )
   (send-super :init)
   (setq _action_event (round __action_event))
   (setq _handle __handle)
   self)
  (:action_event
   (&optional __action_event)
   (if __action_event (setq _action_event __action_event)) _action_event)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:serialization-length
   ()
   (+
    ;; uint32 _action_event
    4
    ;; kortex_driver/ActionHandle _handle
    (send _handle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _action_event
       (write-long _action_event s)
     ;; kortex_driver/ActionHandle _handle
       (send _handle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _action_event
     (setq _action_event (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/ActionHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ActionExecutionState :md5sum-) "ffbf0e1fde1af8a50b9f2f937e17d3ac")
(setf (get kortex_driver::ActionExecutionState :datatype-) "kortex_driver/ActionExecutionState")
(setf (get kortex_driver::ActionExecutionState :definition-)
      "
uint32 action_event
ActionHandle handle
================================================================================
MSG: kortex_driver/ActionHandle

uint32 identifier
uint32 action_type
uint32 permission
")



(provide :kortex_driver/ActionExecutionState "ffbf0e1fde1af8a50b9f2f937e17d3ac")


