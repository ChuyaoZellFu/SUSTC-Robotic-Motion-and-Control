;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetTwistLinearSoftLimit)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetTwistLinearSoftLimit (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETTWISTLINEARSOFTLIMIT")
  (make-package "KORTEX_DRIVER::SETTWISTLINEARSOFTLIMIT"))
(unless (find-package "KORTEX_DRIVER::SETTWISTLINEARSOFTLIMITREQUEST")
  (make-package "KORTEX_DRIVER::SETTWISTLINEARSOFTLIMITREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETTWISTLINEARSOFTLIMITRESPONSE")
  (make-package "KORTEX_DRIVER::SETTWISTLINEARSOFTLIMITRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetTwistLinearSoftLimitRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetTwistLinearSoftLimitRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::TwistLinearSoftLimit :init))
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
    ;; kortex_driver/TwistLinearSoftLimit _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TwistLinearSoftLimit _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TwistLinearSoftLimit _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetTwistLinearSoftLimitResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetTwistLinearSoftLimitResponse
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

(defclass kortex_driver::SetTwistLinearSoftLimit
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetTwistLinearSoftLimit :md5sum-) "1641ba0fe5229f88d9cda0ee47099b0f")
(setf (get kortex_driver::SetTwistLinearSoftLimit :datatype-) "kortex_driver/SetTwistLinearSoftLimit")
(setf (get kortex_driver::SetTwistLinearSoftLimit :request) kortex_driver::SetTwistLinearSoftLimitRequest)
(setf (get kortex_driver::SetTwistLinearSoftLimit :response) kortex_driver::SetTwistLinearSoftLimitResponse)

(defmethod kortex_driver::SetTwistLinearSoftLimitRequest
  (:response () (instance kortex_driver::SetTwistLinearSoftLimitResponse :init)))

(setf (get kortex_driver::SetTwistLinearSoftLimitRequest :md5sum-) "1641ba0fe5229f88d9cda0ee47099b0f")
(setf (get kortex_driver::SetTwistLinearSoftLimitRequest :datatype-) "kortex_driver/SetTwistLinearSoftLimitRequest")
(setf (get kortex_driver::SetTwistLinearSoftLimitRequest :definition-)
      "TwistLinearSoftLimit input

================================================================================
MSG: kortex_driver/TwistLinearSoftLimit

uint32 control_mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetTwistLinearSoftLimitResponse :md5sum-) "1641ba0fe5229f88d9cda0ee47099b0f")
(setf (get kortex_driver::SetTwistLinearSoftLimitResponse :datatype-) "kortex_driver/SetTwistLinearSoftLimitResponse")
(setf (get kortex_driver::SetTwistLinearSoftLimitResponse :definition-)
      "TwistLinearSoftLimit input

================================================================================
MSG: kortex_driver/TwistLinearSoftLimit

uint32 control_mode
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetTwistLinearSoftLimit "1641ba0fe5229f88d9cda0ee47099b0f")


