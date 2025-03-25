;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERSTATE")
  (make-package "KORTEX_DRIVER::CONTROLLERSTATE"))

(in-package "ROS")
;;//! \htmlinclude ControllerState.msg.html


(defclass kortex_driver::ControllerState
  :super ros::object
  :slots (_handle _event_type ))

(defmethod kortex_driver::ControllerState
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::ControllerHandle :init))
    ((:event_type __event_type) 0)
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _event_type (round __event_type))
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
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerHandle _handle
    (send _handle :serialization-length)
    ;; uint32 _event_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerHandle _handle
       (send _handle :serialize s)
     ;; uint32 _event_type
       (write-long _event_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; uint32 _event_type
     (setq _event_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ControllerState :md5sum-) "c93f94c4627606920bff4feca93f2b93")
(setf (get kortex_driver::ControllerState :datatype-) "kortex_driver/ControllerState")
(setf (get kortex_driver::ControllerState :definition-)
      "
ControllerHandle handle
uint32 event_type
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
")



(provide :kortex_driver/ControllerState "c93f94c4627606920bff4feca93f2b93")


