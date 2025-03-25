;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetGPIOState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetGPIOState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETGPIOSTATE")
  (make-package "KORTEX_DRIVER::GETGPIOSTATE"))
(unless (find-package "KORTEX_DRIVER::GETGPIOSTATEREQUEST")
  (make-package "KORTEX_DRIVER::GETGPIOSTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETGPIOSTATERESPONSE")
  (make-package "KORTEX_DRIVER::GETGPIOSTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetGPIOStateRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetGPIOStateRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::GPIOIdentification :init))
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
    ;; kortex_driver/GPIOIdentification _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GPIOIdentification _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GPIOIdentification _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetGPIOStateResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetGPIOStateResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::GPIOState :init))
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
    ;; kortex_driver/GPIOState _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GPIOState _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GPIOState _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetGPIOState
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetGPIOState :md5sum-) "ee1aa8c7cecf549eed7751c07a3eb88c")
(setf (get kortex_driver::GetGPIOState :datatype-) "kortex_driver/GetGPIOState")
(setf (get kortex_driver::GetGPIOState :request) kortex_driver::GetGPIOStateRequest)
(setf (get kortex_driver::GetGPIOState :response) kortex_driver::GetGPIOStateResponse)

(defmethod kortex_driver::GetGPIOStateRequest
  (:response () (instance kortex_driver::GetGPIOStateResponse :init)))

(setf (get kortex_driver::GetGPIOStateRequest :md5sum-) "ee1aa8c7cecf549eed7751c07a3eb88c")
(setf (get kortex_driver::GetGPIOStateRequest :datatype-) "kortex_driver/GetGPIOStateRequest")
(setf (get kortex_driver::GetGPIOStateRequest :definition-)
      "GPIOIdentification input

================================================================================
MSG: kortex_driver/GPIOIdentification

---
GPIOState output

================================================================================
MSG: kortex_driver/GPIOState

uint32 identifier
")

(setf (get kortex_driver::GetGPIOStateResponse :md5sum-) "ee1aa8c7cecf549eed7751c07a3eb88c")
(setf (get kortex_driver::GetGPIOStateResponse :datatype-) "kortex_driver/GetGPIOStateResponse")
(setf (get kortex_driver::GetGPIOStateResponse :definition-)
      "GPIOIdentification input

================================================================================
MSG: kortex_driver/GPIOIdentification

---
GPIOState output

================================================================================
MSG: kortex_driver/GPIOState

uint32 identifier
")



(provide :kortex_driver/GetGPIOState "ee1aa8c7cecf549eed7751c07a3eb88c")


