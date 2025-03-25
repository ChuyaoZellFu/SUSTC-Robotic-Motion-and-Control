;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetActivatedControlLoop)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetActivatedControlLoop (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETACTIVATEDCONTROLLOOP")
  (make-package "KORTEX_DRIVER::SETACTIVATEDCONTROLLOOP"))
(unless (find-package "KORTEX_DRIVER::SETACTIVATEDCONTROLLOOPREQUEST")
  (make-package "KORTEX_DRIVER::SETACTIVATEDCONTROLLOOPREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETACTIVATEDCONTROLLOOPRESPONSE")
  (make-package "KORTEX_DRIVER::SETACTIVATEDCONTROLLOOPRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetActivatedControlLoopRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetActivatedControlLoopRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ControlLoop :init))
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
    ;; kortex_driver/ControlLoop _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControlLoop _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControlLoop _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetActivatedControlLoopResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetActivatedControlLoopResponse
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

(defclass kortex_driver::SetActivatedControlLoop
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetActivatedControlLoop :md5sum-) "0f6f5ae628430443bb031e43516b6adc")
(setf (get kortex_driver::SetActivatedControlLoop :datatype-) "kortex_driver/SetActivatedControlLoop")
(setf (get kortex_driver::SetActivatedControlLoop :request) kortex_driver::SetActivatedControlLoopRequest)
(setf (get kortex_driver::SetActivatedControlLoop :response) kortex_driver::SetActivatedControlLoopResponse)

(defmethod kortex_driver::SetActivatedControlLoopRequest
  (:response () (instance kortex_driver::SetActivatedControlLoopResponse :init)))

(setf (get kortex_driver::SetActivatedControlLoopRequest :md5sum-) "0f6f5ae628430443bb031e43516b6adc")
(setf (get kortex_driver::SetActivatedControlLoopRequest :datatype-) "kortex_driver/SetActivatedControlLoopRequest")
(setf (get kortex_driver::SetActivatedControlLoopRequest :definition-)
      "ControlLoop input

================================================================================
MSG: kortex_driver/ControlLoop

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetActivatedControlLoopResponse :md5sum-) "0f6f5ae628430443bb031e43516b6adc")
(setf (get kortex_driver::SetActivatedControlLoopResponse :datatype-) "kortex_driver/SetActivatedControlLoopResponse")
(setf (get kortex_driver::SetActivatedControlLoopResponse :definition-)
      "ControlLoop input

================================================================================
MSG: kortex_driver/ControlLoop

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetActivatedControlLoop "0f6f5ae628430443bb031e43516b6adc")


