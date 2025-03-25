;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ApplyEmergencyStop)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ApplyEmergencyStop (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::APPLYEMERGENCYSTOP")
  (make-package "KORTEX_DRIVER::APPLYEMERGENCYSTOP"))
(unless (find-package "KORTEX_DRIVER::APPLYEMERGENCYSTOPREQUEST")
  (make-package "KORTEX_DRIVER::APPLYEMERGENCYSTOPREQUEST"))
(unless (find-package "KORTEX_DRIVER::APPLYEMERGENCYSTOPRESPONSE")
  (make-package "KORTEX_DRIVER::APPLYEMERGENCYSTOPRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ApplyEmergencyStopRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ApplyEmergencyStopRequest
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

(defclass kortex_driver::ApplyEmergencyStopResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ApplyEmergencyStopResponse
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

(defclass kortex_driver::ApplyEmergencyStop
  :super ros::object
  :slots ())

(setf (get kortex_driver::ApplyEmergencyStop :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ApplyEmergencyStop :datatype-) "kortex_driver/ApplyEmergencyStop")
(setf (get kortex_driver::ApplyEmergencyStop :request) kortex_driver::ApplyEmergencyStopRequest)
(setf (get kortex_driver::ApplyEmergencyStop :response) kortex_driver::ApplyEmergencyStopResponse)

(defmethod kortex_driver::ApplyEmergencyStopRequest
  (:response () (instance kortex_driver::ApplyEmergencyStopResponse :init)))

(setf (get kortex_driver::ApplyEmergencyStopRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ApplyEmergencyStopRequest :datatype-) "kortex_driver/ApplyEmergencyStopRequest")
(setf (get kortex_driver::ApplyEmergencyStopRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ApplyEmergencyStopResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ApplyEmergencyStopResponse :datatype-) "kortex_driver/ApplyEmergencyStopResponse")
(setf (get kortex_driver::ApplyEmergencyStopResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ApplyEmergencyStop "f335b819dc59099fe3124b36f140ad07")


