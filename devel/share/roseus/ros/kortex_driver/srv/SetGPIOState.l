;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetGPIOState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetGPIOState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETGPIOSTATE")
  (make-package "KORTEX_DRIVER::SETGPIOSTATE"))
(unless (find-package "KORTEX_DRIVER::SETGPIOSTATEREQUEST")
  (make-package "KORTEX_DRIVER::SETGPIOSTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETGPIOSTATERESPONSE")
  (make-package "KORTEX_DRIVER::SETGPIOSTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetGPIOStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetGPIOStateRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::GPIOState :init))
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
    ;; kortex_driver/GPIOState _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GPIOState _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GPIOState _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetGPIOStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetGPIOStateResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetGPIOState
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetGPIOState :md5sum-) "22e0e537e24f27ee3c77b3e42187e849")
(setf (get kortex_driver::SetGPIOState :datatype-) "kortex_driver/SetGPIOState")
(setf (get kortex_driver::SetGPIOState :request) kortex_driver::SetGPIOStateRequest)
(setf (get kortex_driver::SetGPIOState :response) kortex_driver::SetGPIOStateResponse)

(defmethod kortex_driver::SetGPIOStateRequest
  (:response () (instance kortex_driver::SetGPIOStateResponse :init)))

(setf (get kortex_driver::SetGPIOStateRequest :md5sum-) "22e0e537e24f27ee3c77b3e42187e849")
(setf (get kortex_driver::SetGPIOStateRequest :datatype-) "kortex_driver/SetGPIOStateRequest")
(setf (get kortex_driver::SetGPIOStateRequest :definition-)
      "GPIOState input

================================================================================
MSG: kortex_driver/GPIOState

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetGPIOStateResponse :md5sum-) "22e0e537e24f27ee3c77b3e42187e849")
(setf (get kortex_driver::SetGPIOStateResponse :datatype-) "kortex_driver/SetGPIOStateResponse")
(setf (get kortex_driver::SetGPIOStateResponse :definition-)
      "GPIOState input

================================================================================
MSG: kortex_driver/GPIOState

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetGPIOState "22e0e537e24f27ee3c77b3e42187e849")


