;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetGravityVector)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetGravityVector (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETGRAVITYVECTOR")
  (make-package "KORTEX_DRIVER::GETGRAVITYVECTOR"))
(unless (find-package "KORTEX_DRIVER::GETGRAVITYVECTORREQUEST")
  (make-package "KORTEX_DRIVER::GETGRAVITYVECTORREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETGRAVITYVECTORRESPONSE")
  (make-package "KORTEX_DRIVER::GETGRAVITYVECTORRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetGravityVectorRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetGravityVectorRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
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
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetGravityVectorResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetGravityVectorResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::GravityVector :init))
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
    ;; kortex_driver/GravityVector _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GravityVector _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GravityVector _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetGravityVector
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetGravityVector :md5sum-) "8956f0eb2b4baf668929f3e8781781fa")
(setf (get kortex_driver::GetGravityVector :datatype-) "kortex_driver/GetGravityVector")
(setf (get kortex_driver::GetGravityVector :request) kortex_driver::GetGravityVectorRequest)
(setf (get kortex_driver::GetGravityVector :response) kortex_driver::GetGravityVectorResponse)

(defmethod kortex_driver::GetGravityVectorRequest
  (:response () (instance kortex_driver::GetGravityVectorResponse :init)))

(setf (get kortex_driver::GetGravityVectorRequest :md5sum-) "8956f0eb2b4baf668929f3e8781781fa")
(setf (get kortex_driver::GetGravityVectorRequest :datatype-) "kortex_driver/GetGravityVectorRequest")
(setf (get kortex_driver::GetGravityVectorRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
GravityVector output

================================================================================
MSG: kortex_driver/GravityVector

float32 x
float32 y
")

(setf (get kortex_driver::GetGravityVectorResponse :md5sum-) "8956f0eb2b4baf668929f3e8781781fa")
(setf (get kortex_driver::GetGravityVectorResponse :datatype-) "kortex_driver/GetGravityVectorResponse")
(setf (get kortex_driver::GetGravityVectorResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
GravityVector output

================================================================================
MSG: kortex_driver/GravityVector

float32 x
float32 y
")



(provide :kortex_driver/GetGravityVector "8956f0eb2b4baf668929f3e8781781fa")


