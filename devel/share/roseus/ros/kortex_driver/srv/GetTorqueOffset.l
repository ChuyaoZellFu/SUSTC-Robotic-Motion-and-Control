;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetTorqueOffset)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetTorqueOffset (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETTORQUEOFFSET")
  (make-package "KORTEX_DRIVER::GETTORQUEOFFSET"))
(unless (find-package "KORTEX_DRIVER::GETTORQUEOFFSETREQUEST")
  (make-package "KORTEX_DRIVER::GETTORQUEOFFSETREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETTORQUEOFFSETRESPONSE")
  (make-package "KORTEX_DRIVER::GETTORQUEOFFSETRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetTorqueOffsetRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetTorqueOffsetRequest
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

(defclass kortex_driver::GetTorqueOffsetResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetTorqueOffsetResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::TorqueOffset :init))
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
    ;; kortex_driver/TorqueOffset _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/TorqueOffset _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/TorqueOffset _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetTorqueOffset
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetTorqueOffset :md5sum-) "6ed1621e934f6480be9371fb25fd7ea1")
(setf (get kortex_driver::GetTorqueOffset :datatype-) "kortex_driver/GetTorqueOffset")
(setf (get kortex_driver::GetTorqueOffset :request) kortex_driver::GetTorqueOffsetRequest)
(setf (get kortex_driver::GetTorqueOffset :response) kortex_driver::GetTorqueOffsetResponse)

(defmethod kortex_driver::GetTorqueOffsetRequest
  (:response () (instance kortex_driver::GetTorqueOffsetResponse :init)))

(setf (get kortex_driver::GetTorqueOffsetRequest :md5sum-) "6ed1621e934f6480be9371fb25fd7ea1")
(setf (get kortex_driver::GetTorqueOffsetRequest :datatype-) "kortex_driver/GetTorqueOffsetRequest")
(setf (get kortex_driver::GetTorqueOffsetRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
TorqueOffset output

================================================================================
MSG: kortex_driver/TorqueOffset

")

(setf (get kortex_driver::GetTorqueOffsetResponse :md5sum-) "6ed1621e934f6480be9371fb25fd7ea1")
(setf (get kortex_driver::GetTorqueOffsetResponse :datatype-) "kortex_driver/GetTorqueOffsetResponse")
(setf (get kortex_driver::GetTorqueOffsetResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
TorqueOffset output

================================================================================
MSG: kortex_driver/TorqueOffset

")



(provide :kortex_driver/GetTorqueOffset "6ed1621e934f6480be9371fb25fd7ea1")


