;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetServoing)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetServoing (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETSERVOING")
  (make-package "KORTEX_DRIVER::SETSERVOING"))
(unless (find-package "KORTEX_DRIVER::SETSERVOINGREQUEST")
  (make-package "KORTEX_DRIVER::SETSERVOINGREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETSERVOINGRESPONSE")
  (make-package "KORTEX_DRIVER::SETSERVOINGRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetServoingRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetServoingRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Servoing :init))
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
    ;; kortex_driver/Servoing _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Servoing _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Servoing _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetServoingResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetServoingResponse
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

(defclass kortex_driver::SetServoing
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetServoing :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetServoing :datatype-) "kortex_driver/SetServoing")
(setf (get kortex_driver::SetServoing :request) kortex_driver::SetServoingRequest)
(setf (get kortex_driver::SetServoing :response) kortex_driver::SetServoingResponse)

(defmethod kortex_driver::SetServoingRequest
  (:response () (instance kortex_driver::SetServoingResponse :init)))

(setf (get kortex_driver::SetServoingRequest :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetServoingRequest :datatype-) "kortex_driver/SetServoingRequest")
(setf (get kortex_driver::SetServoingRequest :definition-)
      "Servoing input

================================================================================
MSG: kortex_driver/Servoing

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetServoingResponse :md5sum-) "7ccd6a6c56b1642433541444d0e2ab2f")
(setf (get kortex_driver::SetServoingResponse :datatype-) "kortex_driver/SetServoingResponse")
(setf (get kortex_driver::SetServoingResponse :definition-)
      "Servoing input

================================================================================
MSG: kortex_driver/Servoing

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetServoing "7ccd6a6c56b1642433541444d0e2ab2f")


