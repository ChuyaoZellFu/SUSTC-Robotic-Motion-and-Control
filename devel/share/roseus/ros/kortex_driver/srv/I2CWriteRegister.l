;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CWriteRegister)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CWriteRegister (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CWRITEREGISTER")
  (make-package "KORTEX_DRIVER::I2CWRITEREGISTER"))
(unless (find-package "KORTEX_DRIVER::I2CWRITEREGISTERREQUEST")
  (make-package "KORTEX_DRIVER::I2CWRITEREGISTERREQUEST"))
(unless (find-package "KORTEX_DRIVER::I2CWRITEREGISTERRESPONSE")
  (make-package "KORTEX_DRIVER::I2CWRITEREGISTERRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::I2CWriteRegisterRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::I2CWriteRegisterRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::I2CWriteRegisterParameter :init))
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
    ;; kortex_driver/I2CWriteRegisterParameter _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CWriteRegisterParameter _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CWriteRegisterParameter _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::I2CWriteRegisterResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::I2CWriteRegisterResponse
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

(defclass kortex_driver::I2CWriteRegister
  :super ros::object
  :slots ())

(setf (get kortex_driver::I2CWriteRegister :md5sum-) "51d6dc72e2a979ebcd0930e5191f7f87")
(setf (get kortex_driver::I2CWriteRegister :datatype-) "kortex_driver/I2CWriteRegister")
(setf (get kortex_driver::I2CWriteRegister :request) kortex_driver::I2CWriteRegisterRequest)
(setf (get kortex_driver::I2CWriteRegister :response) kortex_driver::I2CWriteRegisterResponse)

(defmethod kortex_driver::I2CWriteRegisterRequest
  (:response () (instance kortex_driver::I2CWriteRegisterResponse :init)))

(setf (get kortex_driver::I2CWriteRegisterRequest :md5sum-) "51d6dc72e2a979ebcd0930e5191f7f87")
(setf (get kortex_driver::I2CWriteRegisterRequest :datatype-) "kortex_driver/I2CWriteRegisterRequest")
(setf (get kortex_driver::I2CWriteRegisterRequest :definition-)
      "I2CWriteRegisterParameter input

================================================================================
MSG: kortex_driver/I2CWriteRegisterParameter

uint32 device
uint32 device_address
uint32 register_address
uint32 register_address_size
uint32 timeout
I2CData data
================================================================================
MSG: kortex_driver/I2CData

uint8[] data
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::I2CWriteRegisterResponse :md5sum-) "51d6dc72e2a979ebcd0930e5191f7f87")
(setf (get kortex_driver::I2CWriteRegisterResponse :datatype-) "kortex_driver/I2CWriteRegisterResponse")
(setf (get kortex_driver::I2CWriteRegisterResponse :definition-)
      "I2CWriteRegisterParameter input

================================================================================
MSG: kortex_driver/I2CWriteRegisterParameter

uint32 device
uint32 device_address
uint32 register_address
uint32 register_address_size
uint32 timeout
I2CData data
================================================================================
MSG: kortex_driver/I2CData

uint8[] data
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/I2CWriteRegister "51d6dc72e2a979ebcd0930e5191f7f87")


