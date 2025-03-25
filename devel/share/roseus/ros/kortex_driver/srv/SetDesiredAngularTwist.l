;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetDesiredAngularTwist)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetDesiredAngularTwist (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETDESIREDANGULARTWIST")
  (make-package "KORTEX_DRIVER::SETDESIREDANGULARTWIST"))
(unless (find-package "KORTEX_DRIVER::SETDESIREDANGULARTWISTREQUEST")
  (make-package "KORTEX_DRIVER::SETDESIREDANGULARTWISTREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETDESIREDANGULARTWISTRESPONSE")
  (make-package "KORTEX_DRIVER::SETDESIREDANGULARTWISTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetDesiredAngularTwistRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetDesiredAngularTwistRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::AngularTwist :init))
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
    ;; kortex_driver/AngularTwist _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/AngularTwist _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/AngularTwist _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetDesiredAngularTwistResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetDesiredAngularTwistResponse
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

(defclass kortex_driver::SetDesiredAngularTwist
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetDesiredAngularTwist :md5sum-) "74bb25440e69b788f4018de65b9d27f9")
(setf (get kortex_driver::SetDesiredAngularTwist :datatype-) "kortex_driver/SetDesiredAngularTwist")
(setf (get kortex_driver::SetDesiredAngularTwist :request) kortex_driver::SetDesiredAngularTwistRequest)
(setf (get kortex_driver::SetDesiredAngularTwist :response) kortex_driver::SetDesiredAngularTwistResponse)

(defmethod kortex_driver::SetDesiredAngularTwistRequest
  (:response () (instance kortex_driver::SetDesiredAngularTwistResponse :init)))

(setf (get kortex_driver::SetDesiredAngularTwistRequest :md5sum-) "74bb25440e69b788f4018de65b9d27f9")
(setf (get kortex_driver::SetDesiredAngularTwistRequest :datatype-) "kortex_driver/SetDesiredAngularTwistRequest")
(setf (get kortex_driver::SetDesiredAngularTwistRequest :definition-)
      "AngularTwist input

================================================================================
MSG: kortex_driver/AngularTwist

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetDesiredAngularTwistResponse :md5sum-) "74bb25440e69b788f4018de65b9d27f9")
(setf (get kortex_driver::SetDesiredAngularTwistResponse :datatype-) "kortex_driver/SetDesiredAngularTwistResponse")
(setf (get kortex_driver::SetDesiredAngularTwistResponse :definition-)
      "AngularTwist input

================================================================================
MSG: kortex_driver/AngularTwist

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetDesiredAngularTwist "74bb25440e69b788f4018de65b9d27f9")


