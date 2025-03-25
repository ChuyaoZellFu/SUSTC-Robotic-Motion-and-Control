;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ResetGravityVector)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ResetGravityVector (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESETGRAVITYVECTOR")
  (make-package "KORTEX_DRIVER::RESETGRAVITYVECTOR"))
(unless (find-package "KORTEX_DRIVER::RESETGRAVITYVECTORREQUEST")
  (make-package "KORTEX_DRIVER::RESETGRAVITYVECTORREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESETGRAVITYVECTORRESPONSE")
  (make-package "KORTEX_DRIVER::RESETGRAVITYVECTORRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ResetGravityVectorRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ResetGravityVectorRequest
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

(defclass kortex_driver::ResetGravityVectorResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ResetGravityVectorResponse
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

(defclass kortex_driver::ResetGravityVector
  :super ros::object
  :slots ())

(setf (get kortex_driver::ResetGravityVector :md5sum-) "8956f0eb2b4baf668929f3e8781781fa")
(setf (get kortex_driver::ResetGravityVector :datatype-) "kortex_driver/ResetGravityVector")
(setf (get kortex_driver::ResetGravityVector :request) kortex_driver::ResetGravityVectorRequest)
(setf (get kortex_driver::ResetGravityVector :response) kortex_driver::ResetGravityVectorResponse)

(defmethod kortex_driver::ResetGravityVectorRequest
  (:response () (instance kortex_driver::ResetGravityVectorResponse :init)))

(setf (get kortex_driver::ResetGravityVectorRequest :md5sum-) "8956f0eb2b4baf668929f3e8781781fa")
(setf (get kortex_driver::ResetGravityVectorRequest :datatype-) "kortex_driver/ResetGravityVectorRequest")
(setf (get kortex_driver::ResetGravityVectorRequest :definition-)
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

(setf (get kortex_driver::ResetGravityVectorResponse :md5sum-) "8956f0eb2b4baf668929f3e8781781fa")
(setf (get kortex_driver::ResetGravityVectorResponse :datatype-) "kortex_driver/ResetGravityVectorResponse")
(setf (get kortex_driver::ResetGravityVectorResponse :definition-)
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



(provide :kortex_driver/ResetGravityVector "8956f0eb2b4baf668929f3e8781781fa")


