;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetModelNumber)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetModelNumber (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETMODELNUMBER")
  (make-package "KORTEX_DRIVER::GETMODELNUMBER"))
(unless (find-package "KORTEX_DRIVER::GETMODELNUMBERREQUEST")
  (make-package "KORTEX_DRIVER::GETMODELNUMBERREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETMODELNUMBERRESPONSE")
  (make-package "KORTEX_DRIVER::GETMODELNUMBERRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetModelNumberRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetModelNumberRequest
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

(defclass kortex_driver::GetModelNumberResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetModelNumberResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::ModelNumber :init))
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
    ;; kortex_driver/ModelNumber _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ModelNumber _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ModelNumber _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetModelNumber
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetModelNumber :md5sum-) "af635fd95ef990bb7c96e0afad0126ac")
(setf (get kortex_driver::GetModelNumber :datatype-) "kortex_driver/GetModelNumber")
(setf (get kortex_driver::GetModelNumber :request) kortex_driver::GetModelNumberRequest)
(setf (get kortex_driver::GetModelNumber :response) kortex_driver::GetModelNumberResponse)

(defmethod kortex_driver::GetModelNumberRequest
  (:response () (instance kortex_driver::GetModelNumberResponse :init)))

(setf (get kortex_driver::GetModelNumberRequest :md5sum-) "af635fd95ef990bb7c96e0afad0126ac")
(setf (get kortex_driver::GetModelNumberRequest :datatype-) "kortex_driver/GetModelNumberRequest")
(setf (get kortex_driver::GetModelNumberRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ModelNumber output

================================================================================
MSG: kortex_driver/ModelNumber

")

(setf (get kortex_driver::GetModelNumberResponse :md5sum-) "af635fd95ef990bb7c96e0afad0126ac")
(setf (get kortex_driver::GetModelNumberResponse :datatype-) "kortex_driver/GetModelNumberResponse")
(setf (get kortex_driver::GetModelNumberResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
ModelNumber output

================================================================================
MSG: kortex_driver/ModelNumber

")



(provide :kortex_driver/GetModelNumber "af635fd95ef990bb7c96e0afad0126ac")


