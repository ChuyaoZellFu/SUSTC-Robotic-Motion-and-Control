;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetControllerState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetControllerState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERSTATE")
  (make-package "KORTEX_DRIVER::GETCONTROLLERSTATE"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERSTATEREQUEST")
  (make-package "KORTEX_DRIVER::GETCONTROLLERSTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETCONTROLLERSTATERESPONSE")
  (make-package "KORTEX_DRIVER::GETCONTROLLERSTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetControllerStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetControllerStateRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControllerHandle :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetControllerStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetControllerStateResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ControllerState :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerState _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerState _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerState _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetControllerState
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetControllerState :md5sum-) "c3a3b583b8a6198c6f61bb7c1bde10f0")
(setf (get kortex_driver::GetControllerState :datatype-) "kortex_driver/GetControllerState")
(setf (get kortex_driver::GetControllerState :request) kortex_driver::GetControllerStateRequest)
(setf (get kortex_driver::GetControllerState :response) kortex_driver::GetControllerStateResponse)

(defmethod kortex_driver::GetControllerStateRequest
  (:response () (instance kortex_driver::GetControllerStateResponse :init)))

(setf (get kortex_driver::GetControllerStateRequest :md5sum-) "c3a3b583b8a6198c6f61bb7c1bde10f0")
(setf (get kortex_driver::GetControllerStateRequest :datatype-) "kortex_driver/GetControllerStateRequest")
(setf (get kortex_driver::GetControllerStateRequest :definition-)
      "ControllerHandle input

================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
---
ControllerState output

================================================================================
MSG: kortex_driver/ControllerState

ControllerHandle handle
uint32 event_type
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
")

(setf (get kortex_driver::GetControllerStateResponse :md5sum-) "c3a3b583b8a6198c6f61bb7c1bde10f0")
(setf (get kortex_driver::GetControllerStateResponse :datatype-) "kortex_driver/GetControllerStateResponse")
(setf (get kortex_driver::GetControllerStateResponse :definition-)
      "ControllerHandle input

================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
---
ControllerState output

================================================================================
MSG: kortex_driver/ControllerState

ControllerHandle handle
uint32 event_type
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
")



(provide :kortex_driver/GetControllerState "c3a3b583b8a6198c6f61bb7c1bde10f0")


