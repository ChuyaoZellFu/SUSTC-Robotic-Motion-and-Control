;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CReadRegister)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CReadRegister (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CREADREGISTER")
  (make-package "KORTEX_DRIVER::I2CREADREGISTER"))
(unless (find-package "KORTEX_DRIVER::I2CREADREGISTERREQUEST")
  (make-package "KORTEX_DRIVER::I2CREADREGISTERREQUEST"))
(unless (find-package "KORTEX_DRIVER::I2CREADREGISTERRESPONSE")
  (make-package "KORTEX_DRIVER::I2CREADREGISTERRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::I2CReadRegisterRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::I2CReadRegisterRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::I2CReadRegisterParameter :init))
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
    ;; kortex_driver/I2CReadRegisterParameter _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CReadRegisterParameter _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CReadRegisterParameter _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::I2CReadRegisterResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::I2CReadRegisterResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::I2CData :init))
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
    ;; kortex_driver/I2CData _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CData _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CData _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::I2CReadRegister
  :super ros::object
  :slots ())

(setf (get kortex_driver::I2CReadRegister :md5sum-) "4bdf02ffe8fd0ef6ff8749422c787f42")
(setf (get kortex_driver::I2CReadRegister :datatype-) "kortex_driver/I2CReadRegister")
(setf (get kortex_driver::I2CReadRegister :request) kortex_driver::I2CReadRegisterRequest)
(setf (get kortex_driver::I2CReadRegister :response) kortex_driver::I2CReadRegisterResponse)

(defmethod kortex_driver::I2CReadRegisterRequest
  (:response () (instance kortex_driver::I2CReadRegisterResponse :init)))

(setf (get kortex_driver::I2CReadRegisterRequest :md5sum-) "4bdf02ffe8fd0ef6ff8749422c787f42")
(setf (get kortex_driver::I2CReadRegisterRequest :datatype-) "kortex_driver/I2CReadRegisterRequest")
(setf (get kortex_driver::I2CReadRegisterRequest :definition-)
      "I2CReadRegisterParameter input

================================================================================
MSG: kortex_driver/I2CReadRegisterParameter

uint32 device
uint32 device_address
uint32 register_address
uint32 register_address_size
uint32 size
---
I2CData output

================================================================================
MSG: kortex_driver/I2CData

uint8[] data
")

(setf (get kortex_driver::I2CReadRegisterResponse :md5sum-) "4bdf02ffe8fd0ef6ff8749422c787f42")
(setf (get kortex_driver::I2CReadRegisterResponse :datatype-) "kortex_driver/I2CReadRegisterResponse")
(setf (get kortex_driver::I2CReadRegisterResponse :definition-)
      "I2CReadRegisterParameter input

================================================================================
MSG: kortex_driver/I2CReadRegisterParameter

uint32 device
uint32 device_address
uint32 register_address
uint32 register_address_size
uint32 size
---
I2CData output

================================================================================
MSG: kortex_driver/I2CData

uint8[] data
")



(provide :kortex_driver/I2CReadRegister "4bdf02ffe8fd0ef6ff8749422c787f42")


