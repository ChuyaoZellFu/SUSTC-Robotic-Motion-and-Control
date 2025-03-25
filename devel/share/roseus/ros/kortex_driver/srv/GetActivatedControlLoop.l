;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetActivatedControlLoop)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetActivatedControlLoop (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETACTIVATEDCONTROLLOOP")
  (make-package "KORTEX_DRIVER::GETACTIVATEDCONTROLLOOP"))
(unless (find-package "KORTEX_DRIVER::GETACTIVATEDCONTROLLOOPREQUEST")
  (make-package "KORTEX_DRIVER::GETACTIVATEDCONTROLLOOPREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETACTIVATEDCONTROLLOOPRESPONSE")
  (make-package "KORTEX_DRIVER::GETACTIVATEDCONTROLLOOPRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetActivatedControlLoopRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetActivatedControlLoopRequest
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

(defclass kortex_driver::GetActivatedControlLoopResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetActivatedControlLoopResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ControlLoop :init))
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
    ;; kortex_driver/ControlLoop _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControlLoop _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControlLoop _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetActivatedControlLoop
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetActivatedControlLoop :md5sum-) "acf43fee1c712ad0288037798483900b")
(setf (get kortex_driver::GetActivatedControlLoop :datatype-) "kortex_driver/GetActivatedControlLoop")
(setf (get kortex_driver::GetActivatedControlLoop :request) kortex_driver::GetActivatedControlLoopRequest)
(setf (get kortex_driver::GetActivatedControlLoop :response) kortex_driver::GetActivatedControlLoopResponse)

(defmethod kortex_driver::GetActivatedControlLoopRequest
  (:response () (instance kortex_driver::GetActivatedControlLoopResponse :init)))

(setf (get kortex_driver::GetActivatedControlLoopRequest :md5sum-) "acf43fee1c712ad0288037798483900b")
(setf (get kortex_driver::GetActivatedControlLoopRequest :datatype-) "kortex_driver/GetActivatedControlLoopRequest")
(setf (get kortex_driver::GetActivatedControlLoopRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControlLoop output

================================================================================
MSG: kortex_driver/ControlLoop

")

(setf (get kortex_driver::GetActivatedControlLoopResponse :md5sum-) "acf43fee1c712ad0288037798483900b")
(setf (get kortex_driver::GetActivatedControlLoopResponse :datatype-) "kortex_driver/GetActivatedControlLoopResponse")
(setf (get kortex_driver::GetActivatedControlLoopResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ControlLoop output

================================================================================
MSG: kortex_driver/ControlLoop

")



(provide :kortex_driver/GetActivatedControlLoop "acf43fee1c712ad0288037798483900b")


