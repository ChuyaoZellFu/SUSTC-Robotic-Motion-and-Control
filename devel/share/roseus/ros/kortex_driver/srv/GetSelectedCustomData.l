;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetSelectedCustomData)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetSelectedCustomData (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETSELECTEDCUSTOMDATA")
  (make-package "KORTEX_DRIVER::GETSELECTEDCUSTOMDATA"))
(unless (find-package "KORTEX_DRIVER::GETSELECTEDCUSTOMDATAREQUEST")
  (make-package "KORTEX_DRIVER::GETSELECTEDCUSTOMDATAREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETSELECTEDCUSTOMDATARESPONSE")
  (make-package "KORTEX_DRIVER::GETSELECTEDCUSTOMDATARESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetSelectedCustomDataRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetSelectedCustomDataRequest
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

(defclass kortex_driver::GetSelectedCustomDataResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetSelectedCustomDataResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::CustomDataSelection :init))
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
    ;; kortex_driver/CustomDataSelection _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CustomDataSelection _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CustomDataSelection _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetSelectedCustomData
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetSelectedCustomData :md5sum-) "d4420814b14d58afc7b7b98f4fdd7907")
(setf (get kortex_driver::GetSelectedCustomData :datatype-) "kortex_driver/GetSelectedCustomData")
(setf (get kortex_driver::GetSelectedCustomData :request) kortex_driver::GetSelectedCustomDataRequest)
(setf (get kortex_driver::GetSelectedCustomData :response) kortex_driver::GetSelectedCustomDataResponse)

(defmethod kortex_driver::GetSelectedCustomDataRequest
  (:response () (instance kortex_driver::GetSelectedCustomDataResponse :init)))

(setf (get kortex_driver::GetSelectedCustomDataRequest :md5sum-) "d4420814b14d58afc7b7b98f4fdd7907")
(setf (get kortex_driver::GetSelectedCustomDataRequest :datatype-) "kortex_driver/GetSelectedCustomDataRequest")
(setf (get kortex_driver::GetSelectedCustomDataRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CustomDataSelection output

================================================================================
MSG: kortex_driver/CustomDataSelection

")

(setf (get kortex_driver::GetSelectedCustomDataResponse :md5sum-) "d4420814b14d58afc7b7b98f4fdd7907")
(setf (get kortex_driver::GetSelectedCustomDataResponse :datatype-) "kortex_driver/GetSelectedCustomDataResponse")
(setf (get kortex_driver::GetSelectedCustomDataResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
CustomDataSelection output

================================================================================
MSG: kortex_driver/CustomDataSelection

")



(provide :kortex_driver/GetSelectedCustomData "d4420814b14d58afc7b7b98f4fdd7907")


