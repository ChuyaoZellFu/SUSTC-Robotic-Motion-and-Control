;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetGravityVector)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetGravityVector (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETGRAVITYVECTOR")
  (make-package "KORTEX_DRIVER::SETGRAVITYVECTOR"))
(unless (find-package "KORTEX_DRIVER::SETGRAVITYVECTORREQUEST")
  (make-package "KORTEX_DRIVER::SETGRAVITYVECTORREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETGRAVITYVECTORRESPONSE")
  (make-package "KORTEX_DRIVER::SETGRAVITYVECTORRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetGravityVectorRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetGravityVectorRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::GravityVector :init))
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
    ;; kortex_driver/GravityVector _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GravityVector _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GravityVector _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetGravityVectorResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetGravityVectorResponse
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

(defclass kortex_driver::SetGravityVector
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetGravityVector :md5sum-) "0af2ee5f9fbb095072dc9d9e57b9318b")
(setf (get kortex_driver::SetGravityVector :datatype-) "kortex_driver/SetGravityVector")
(setf (get kortex_driver::SetGravityVector :request) kortex_driver::SetGravityVectorRequest)
(setf (get kortex_driver::SetGravityVector :response) kortex_driver::SetGravityVectorResponse)

(defmethod kortex_driver::SetGravityVectorRequest
  (:response () (instance kortex_driver::SetGravityVectorResponse :init)))

(setf (get kortex_driver::SetGravityVectorRequest :md5sum-) "0af2ee5f9fbb095072dc9d9e57b9318b")
(setf (get kortex_driver::SetGravityVectorRequest :datatype-) "kortex_driver/SetGravityVectorRequest")
(setf (get kortex_driver::SetGravityVectorRequest :definition-)
      "GravityVector input

================================================================================
MSG: kortex_driver/GravityVector

float32 x
float32 y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetGravityVectorResponse :md5sum-) "0af2ee5f9fbb095072dc9d9e57b9318b")
(setf (get kortex_driver::SetGravityVectorResponse :datatype-) "kortex_driver/SetGravityVectorResponse")
(setf (get kortex_driver::SetGravityVectorResponse :definition-)
      "GravityVector input

================================================================================
MSG: kortex_driver/GravityVector

float32 x
float32 y
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetGravityVector "0af2ee5f9fbb095072dc9d9e57b9318b")


