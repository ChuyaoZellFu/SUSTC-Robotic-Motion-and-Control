;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CWrite)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CWrite (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CWRITE")
  (make-package "KORTEX_DRIVER::I2CWRITE"))
(unless (find-package "KORTEX_DRIVER::I2CWRITEREQUEST")
  (make-package "KORTEX_DRIVER::I2CWRITEREQUEST"))
(unless (find-package "KORTEX_DRIVER::I2CWRITERESPONSE")
  (make-package "KORTEX_DRIVER::I2CWRITERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::I2CWriteRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::I2CWriteRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::I2CWriteParameter :init))
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
    ;; kortex_driver/I2CWriteParameter _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CWriteParameter _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CWriteParameter _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::I2CWriteResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::I2CWriteResponse
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

(defclass kortex_driver::I2CWrite
  :super ros::object
  :slots ())

(setf (get kortex_driver::I2CWrite :md5sum-) "976795fac6276cbe1f1d899c54c1e043")
(setf (get kortex_driver::I2CWrite :datatype-) "kortex_driver/I2CWrite")
(setf (get kortex_driver::I2CWrite :request) kortex_driver::I2CWriteRequest)
(setf (get kortex_driver::I2CWrite :response) kortex_driver::I2CWriteResponse)

(defmethod kortex_driver::I2CWriteRequest
  (:response () (instance kortex_driver::I2CWriteResponse :init)))

(setf (get kortex_driver::I2CWriteRequest :md5sum-) "976795fac6276cbe1f1d899c54c1e043")
(setf (get kortex_driver::I2CWriteRequest :datatype-) "kortex_driver/I2CWriteRequest")
(setf (get kortex_driver::I2CWriteRequest :definition-)
      "I2CWriteParameter input

================================================================================
MSG: kortex_driver/I2CWriteParameter

uint32 device
uint32 device_address
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

(setf (get kortex_driver::I2CWriteResponse :md5sum-) "976795fac6276cbe1f1d899c54c1e043")
(setf (get kortex_driver::I2CWriteResponse :datatype-) "kortex_driver/I2CWriteResponse")
(setf (get kortex_driver::I2CWriteResponse :definition-)
      "I2CWriteParameter input

================================================================================
MSG: kortex_driver/I2CWriteParameter

uint32 device
uint32 device_address
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



(provide :kortex_driver/I2CWrite "976795fac6276cbe1f1d899c54c1e043")


