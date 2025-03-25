;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetDesiredLinearTwist)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetDesiredLinearTwist (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETDESIREDLINEARTWIST")
  (make-package "KORTEX_DRIVER::SETDESIREDLINEARTWIST"))
(unless (find-package "KORTEX_DRIVER::SETDESIREDLINEARTWISTREQUEST")
  (make-package "KORTEX_DRIVER::SETDESIREDLINEARTWISTREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETDESIREDLINEARTWISTRESPONSE")
  (make-package "KORTEX_DRIVER::SETDESIREDLINEARTWISTRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetDesiredLinearTwistRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetDesiredLinearTwistRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::LinearTwist :init))
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
    ;; kortex_driver/LinearTwist _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/LinearTwist _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/LinearTwist _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetDesiredLinearTwistResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetDesiredLinearTwistResponse
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

(defclass kortex_driver::SetDesiredLinearTwist
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetDesiredLinearTwist :md5sum-) "2fd07256b791580cac358c271b7f0f2a")
(setf (get kortex_driver::SetDesiredLinearTwist :datatype-) "kortex_driver/SetDesiredLinearTwist")
(setf (get kortex_driver::SetDesiredLinearTwist :request) kortex_driver::SetDesiredLinearTwistRequest)
(setf (get kortex_driver::SetDesiredLinearTwist :response) kortex_driver::SetDesiredLinearTwistResponse)

(defmethod kortex_driver::SetDesiredLinearTwistRequest
  (:response () (instance kortex_driver::SetDesiredLinearTwistResponse :init)))

(setf (get kortex_driver::SetDesiredLinearTwistRequest :md5sum-) "2fd07256b791580cac358c271b7f0f2a")
(setf (get kortex_driver::SetDesiredLinearTwistRequest :datatype-) "kortex_driver/SetDesiredLinearTwistRequest")
(setf (get kortex_driver::SetDesiredLinearTwistRequest :definition-)
      "LinearTwist input

================================================================================
MSG: kortex_driver/LinearTwist

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetDesiredLinearTwistResponse :md5sum-) "2fd07256b791580cac358c271b7f0f2a")
(setf (get kortex_driver::SetDesiredLinearTwistResponse :datatype-) "kortex_driver/SetDesiredLinearTwistResponse")
(setf (get kortex_driver::SetDesiredLinearTwistResponse :definition-)
      "LinearTwist input

================================================================================
MSG: kortex_driver/LinearTwist

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetDesiredLinearTwist "2fd07256b791580cac358c271b7f0f2a")


