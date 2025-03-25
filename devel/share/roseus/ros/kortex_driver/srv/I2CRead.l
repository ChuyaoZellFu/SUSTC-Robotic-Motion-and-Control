;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CRead)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CRead (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CREAD")
  (make-package "KORTEX_DRIVER::I2CREAD"))
(unless (find-package "KORTEX_DRIVER::I2CREADREQUEST")
  (make-package "KORTEX_DRIVER::I2CREADREQUEST"))
(unless (find-package "KORTEX_DRIVER::I2CREADRESPONSE")
  (make-package "KORTEX_DRIVER::I2CREADRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::I2CReadRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::I2CReadRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::I2CReadParameter :init))
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
    ;; kortex_driver/I2CReadParameter _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/I2CReadParameter _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/I2CReadParameter _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::I2CReadResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::I2CReadResponse
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

(defclass kortex_driver::I2CRead
  :super ros::object
  :slots ())

(setf (get kortex_driver::I2CRead :md5sum-) "76252f712912deacb218cb19eaf79993")
(setf (get kortex_driver::I2CRead :datatype-) "kortex_driver/I2CRead")
(setf (get kortex_driver::I2CRead :request) kortex_driver::I2CReadRequest)
(setf (get kortex_driver::I2CRead :response) kortex_driver::I2CReadResponse)

(defmethod kortex_driver::I2CReadRequest
  (:response () (instance kortex_driver::I2CReadResponse :init)))

(setf (get kortex_driver::I2CReadRequest :md5sum-) "76252f712912deacb218cb19eaf79993")
(setf (get kortex_driver::I2CReadRequest :datatype-) "kortex_driver/I2CReadRequest")
(setf (get kortex_driver::I2CReadRequest :definition-)
      "I2CReadParameter input

================================================================================
MSG: kortex_driver/I2CReadParameter

uint32 device
uint32 device_address
uint32 size
---
I2CData output

================================================================================
MSG: kortex_driver/I2CData

uint8[] data
")

(setf (get kortex_driver::I2CReadResponse :md5sum-) "76252f712912deacb218cb19eaf79993")
(setf (get kortex_driver::I2CReadResponse :datatype-) "kortex_driver/I2CReadResponse")
(setf (get kortex_driver::I2CReadResponse :definition-)
      "I2CReadParameter input

================================================================================
MSG: kortex_driver/I2CReadParameter

uint32 device
uint32 device_address
uint32 size
---
I2CData output

================================================================================
MSG: kortex_driver/I2CData

uint8[] data
")



(provide :kortex_driver/I2CRead "76252f712912deacb218cb19eaf79993")


