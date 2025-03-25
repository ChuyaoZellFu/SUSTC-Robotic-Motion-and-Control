;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SetAdmittance)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SetAdmittance (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SETADMITTANCE")
  (make-package "KORTEX_DRIVER::SETADMITTANCE"))
(unless (find-package "KORTEX_DRIVER::SETADMITTANCEREQUEST")
  (make-package "KORTEX_DRIVER::SETADMITTANCEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SETADMITTANCERESPONSE")
  (make-package "KORTEX_DRIVER::SETADMITTANCERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SetAdmittanceRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::SetAdmittanceRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Admittance :init))
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
    ;; kortex_driver/Admittance _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Admittance _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Admittance _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::SetAdmittanceResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::SetAdmittanceResponse
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

(defclass kortex_driver::SetAdmittance
  :super ros::object
  :slots ())

(setf (get kortex_driver::SetAdmittance :md5sum-) "3af9f43d4260b726067035bc086c02b7")
(setf (get kortex_driver::SetAdmittance :datatype-) "kortex_driver/SetAdmittance")
(setf (get kortex_driver::SetAdmittance :request) kortex_driver::SetAdmittanceRequest)
(setf (get kortex_driver::SetAdmittance :response) kortex_driver::SetAdmittanceResponse)

(defmethod kortex_driver::SetAdmittanceRequest
  (:response () (instance kortex_driver::SetAdmittanceResponse :init)))

(setf (get kortex_driver::SetAdmittanceRequest :md5sum-) "3af9f43d4260b726067035bc086c02b7")
(setf (get kortex_driver::SetAdmittanceRequest :datatype-) "kortex_driver/SetAdmittanceRequest")
(setf (get kortex_driver::SetAdmittanceRequest :definition-)
      "Admittance input

================================================================================
MSG: kortex_driver/Admittance

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::SetAdmittanceResponse :md5sum-) "3af9f43d4260b726067035bc086c02b7")
(setf (get kortex_driver::SetAdmittanceResponse :datatype-) "kortex_driver/SetAdmittanceResponse")
(setf (get kortex_driver::SetAdmittanceResponse :definition-)
      "Admittance input

================================================================================
MSG: kortex_driver/Admittance

---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/SetAdmittance "3af9f43d4260b726067035bc086c02b7")


