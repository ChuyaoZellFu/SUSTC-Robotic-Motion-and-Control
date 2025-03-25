;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetTorqueOffset)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetTorqueOffset (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETTORQUEOFFSET")
  (make-package "KORTEX_DRIVER::SETTORQUEOFFSET"))
(unless (find-package "KORTEX_DRIVER::SETTORQUEOFFSETREQUEST")
  (make-package "KORTEX_DRIVER::SETTORQUEOFFSETREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETTORQUEOFFSETRESPONSE")
  (make-package "KORTEX_DRIVER::SETTORQUEOFFSETRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetTorqueOffsetRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetTorqueOffsetRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::TorqueOffset :init))
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
    ;; kortex_driver/TorqueOffset _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TorqueOffset _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TorqueOffset _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetTorqueOffsetResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetTorqueOffsetResponse
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

(defclass kortex_driver::SetTorqueOffset
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetTorqueOffset :md5sum-) "8732fa2318b2bb30f603b1c319396cd0")
(setf (get kortex_driver::SetTorqueOffset :datatype-) "kortex_driver/SetTorqueOffset")
(setf (get kortex_driver::SetTorqueOffset :request) kortex_driver::SetTorqueOffsetRequest)
(setf (get kortex_driver::SetTorqueOffset :response) kortex_driver::SetTorqueOffsetResponse)

(defmethod kortex_driver::SetTorqueOffsetRequest
  (:response () (instance kortex_driver::SetTorqueOffsetResponse :init)))

(setf (get kortex_driver::SetTorqueOffsetRequest :md5sum-) "8732fa2318b2bb30f603b1c319396cd0")
(setf (get kortex_driver::SetTorqueOffsetRequest :datatype-) "kortex_driver/SetTorqueOffsetRequest")
(setf (get kortex_driver::SetTorqueOffsetRequest :definition-)
      "TorqueOffset input

================================================================================
MSG: kortex_driver/TorqueOffset

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetTorqueOffsetResponse :md5sum-) "8732fa2318b2bb30f603b1c319396cd0")
(setf (get kortex_driver::SetTorqueOffsetResponse :datatype-) "kortex_driver/SetTorqueOffsetResponse")
(setf (get kortex_driver::SetTorqueOffsetResponse :definition-)
      "TorqueOffset input

================================================================================
MSG: kortex_driver/TorqueOffset

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetTorqueOffset "8732fa2318b2bb30f603b1c319396cd0")


