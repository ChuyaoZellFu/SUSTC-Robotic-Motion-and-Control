;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ClearSafetyStatus)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ClearSafetyStatus (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CLEARSAFETYSTATUS")
  (make-package "KORTEX_DRIVER::CLEARSAFETYSTATUS"))
(unless (find-package "KORTEX_DRIVER::CLEARSAFETYSTATUSREQUEST")
  (make-package "KORTEX_DRIVER::CLEARSAFETYSTATUSREQUEST"))
(unless (find-package "KORTEX_DRIVER::CLEARSAFETYSTATUSRESPONSE")
  (make-package "KORTEX_DRIVER::CLEARSAFETYSTATUSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ClearSafetyStatusRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::ClearSafetyStatusRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::SafetyHandle :init))
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
    ;; kortex_driver/SafetyHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::ClearSafetyStatusResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::ClearSafetyStatusResponse
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

(defclass kortex_driver::ClearSafetyStatus
  :super ros::object
  :slots ())

(setf (get kortex_driver::ClearSafetyStatus :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::ClearSafetyStatus :datatype-) "kortex_driver/ClearSafetyStatus")
(setf (get kortex_driver::ClearSafetyStatus :request) kortex_driver::ClearSafetyStatusRequest)
(setf (get kortex_driver::ClearSafetyStatus :response) kortex_driver::ClearSafetyStatusResponse)

(defmethod kortex_driver::ClearSafetyStatusRequest
  (:response () (instance kortex_driver::ClearSafetyStatusResponse :init)))

(setf (get kortex_driver::ClearSafetyStatusRequest :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::ClearSafetyStatusRequest :datatype-) "kortex_driver/ClearSafetyStatusRequest")
(setf (get kortex_driver::ClearSafetyStatusRequest :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::ClearSafetyStatusResponse :md5sum-) "7960dab80443332660b3bb4bf774c9ab")
(setf (get kortex_driver::ClearSafetyStatusResponse :datatype-) "kortex_driver/ClearSafetyStatusResponse")
(setf (get kortex_driver::ClearSafetyStatusResponse :definition-)
      "SafetyHandle input

================================================================================
MSG: kortex_driver/SafetyHandle

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/ClearSafetyStatus "7960dab80443332660b3bb4bf774c9ab")


