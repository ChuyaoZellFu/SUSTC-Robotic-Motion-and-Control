;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ClearAllSafetyStatus)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ClearAllSafetyStatus (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CLEARALLSAFETYSTATUS")
  (make-package "KORTEX_DRIVER::CLEARALLSAFETYSTATUS"))
(unless (find-package "KORTEX_DRIVER::CLEARALLSAFETYSTATUSREQUEST")
  (make-package "KORTEX_DRIVER::CLEARALLSAFETYSTATUSREQUEST"))
(unless (find-package "KORTEX_DRIVER::CLEARALLSAFETYSTATUSRESPONSE")
  (make-package "KORTEX_DRIVER::CLEARALLSAFETYSTATUSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ClearAllSafetyStatusRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ClearAllSafetyStatusRequest
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

(defclass kortex_driver::ClearAllSafetyStatusResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ClearAllSafetyStatusResponse
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

(defclass kortex_driver::ClearAllSafetyStatus
  :super ros::object
  :slots ())

(setf (get kortex_driver::ClearAllSafetyStatus :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ClearAllSafetyStatus :datatype-) "kortex_driver/ClearAllSafetyStatus")
(setf (get kortex_driver::ClearAllSafetyStatus :request) kortex_driver::ClearAllSafetyStatusRequest)
(setf (get kortex_driver::ClearAllSafetyStatus :response) kortex_driver::ClearAllSafetyStatusResponse)

(defmethod kortex_driver::ClearAllSafetyStatusRequest
  (:response () (instance kortex_driver::ClearAllSafetyStatusResponse :init)))

(setf (get kortex_driver::ClearAllSafetyStatusRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ClearAllSafetyStatusRequest :datatype-) "kortex_driver/ClearAllSafetyStatusRequest")
(setf (get kortex_driver::ClearAllSafetyStatusRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ClearAllSafetyStatusResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::ClearAllSafetyStatusResponse :datatype-) "kortex_driver/ClearAllSafetyStatusResponse")
(setf (get kortex_driver::ClearAllSafetyStatusResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ClearAllSafetyStatus "f335b819dc59099fe3124b36f140ad07")


